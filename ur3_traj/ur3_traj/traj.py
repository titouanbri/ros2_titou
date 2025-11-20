#!/usr/bin/env python
import sys
import copy
import time
import numpy as np
import math
from typing import Tuple
from collections import deque
from threading import Thread

# Imports ROS 2
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Imports MoveIt
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

# Imports de messages et services
from ur_msgs.srv import SetIO
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3, Quaternion, Pose
from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectoryPoint

# Imports pour TF et maths
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R
from scipy.signal import butter, lfilter_zi, lfilter, filtfilt

# Compatibilité math (dist, tau) - Python 3.8+ a tout
from math import pi, tau, dist, fabs, cos


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self, node: Node):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        self.node = node
        self.logger = self.node.get_logger()

        # Initialisation de MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Publisher pour RViz
        self.display_trajectory_publisher = self.node.create_publisher(
            moveit_msgs.msg.DisplayTrajectory,
            "/move_group/display_planned_path",
            10,
        )

        # Infos de base
        self.planning_frame = self.move_group.get_planning_frame()
        self.tool0 = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        
        self.logger.info("============ Planning frame: %s" % self.planning_frame)
        self.logger.info("============ End effector link: %s" % self.tool0)
        self.logger.info("============ Available Planning Groups: %s" % self.group_names)
        self.logger.info("============ Printing robot state")
        self.logger.info(str(self.robot.get_current_state()))

        # Initialisation TF2
        self.tf_buffer = tf2_ros.Buffer(self.node.get_clock())
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # Clients de service
        self.switch_controller_client = self.node.create_client(
            SwitchController, '/controller_manager/switch_controller')
            
        self.set_io_client = self.node.create_client(
            SetIO, '/ur_hardware_interface/set_io')

        # Variables de classe
        self.box_name = ""
        self.force_data = None
        self.last_R_mat = None
        self.last_Rot = None


   
    def go_to_pose_goal(self,x,y,z,roll, pitch, yaw):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        
        i,j,k,w = self.euler_to_quaternion(roll,pitch,yaw)
        pose_goal.orientation.x = i
        pose_goal.orientation.y = j
        pose_goal.orientation.z = k
        pose_goal.orientation.w = w

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        move_group.set_pose_target(pose_goal)
        
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self,a,b,c):
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x += a  
        wpose.position.y += b  
        wpose.position.z += c
        waypoints.append(copy.deepcopy(wpose))
        
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01  # eef_step
        )
        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

    def get_current_time_sec(self):
        """Obtient le temps actuel en secondes (float)"""
        now = self.node.get_clock().now().to_msg()
        return now.sec + now.nanosec / 1e9

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        box_name = self.box_name
        scene = self.scene

        start = self.get_current_time_sec()
        seconds = self.get_current_time_sec()
        while (seconds - start < timeout) and rclpy.ok():
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = box_name in scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            time.sleep(0.1) # Remplacement de rospy.sleep
            seconds = self.get_current_time_sec()
        return False
        


    def traj1(self,pose_0,angle,gripper_L,nb_point):
        waypoints=[]
        pose_0=np.array(pose_0)
        pose_safe=pose_0-np.array([0.05,0.05,0.05,0,0,0])
        self.go_to_pose_goal(*pose_safe)

        for n in range (nb_point):
            pose=Pose()
            teta=2*np.pi*n/nb_point
            roll=pose_0[3] + np.sin(teta)*angle
            pitch=pose_0[4] + np.cos(teta)*angle
            yaw=pose_0[5]

            x=pose_0[0]
            y=pose_0[1]+gripper_L*np.sin(angle)*np.sin(teta)
            z=pose_0[2]+gripper_L*np.sin(angle)*np.cos(teta)

            qx,qy,qz,qw=self.euler_to_quaternion(roll,pitch,yaw)
            pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            pose.position.x,pose.position.y,pose.position.z = x,y,z
            waypoints.append(pose)

        move_group = self.move_group
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01  # eef_step
        )
        self.logger.info(f"Trajectoire planifiée à {fraction*100:.1f}% des waypoints")
        robot = self.robot
        
        if fraction > 0.99:
            self.display_trajectory(plan)
            self.move_group.execute(plan, wait=True)
        else:
            self.logger.warn("Planification incomplète, ajuste eef_step / rayon_angle / nb_point")      


def main():
    rclpy.init(args=sys.argv)
    
    # Crée le nœud
    node = rclpy.create_node("sensor_test_script")
    
    # Crée un exécuteur et un thread pour le "spinner"
    # C'est crucial pour que les services et MoveIt fonctionnent en arrière-plan
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Initialise moveit_commander (il utilisera le contexte rclpy déjà initialisé)
    moveit_commander.roscpp_initialize(node.get_logger())
    
    try:
        tutorial = MoveGroupPythonInterfaceTutorial(node)
        
        node.get_logger().info("t'as pas le temps de lire de toute façon")
      
        input("============ Press `Enter` to start the movement ...")
        tutorial.traj([0.579,0.39,0.272],[-1,0,0.25],0.25,0.40,50)
        
        
        input("============ Press `Enter` to go to the head ...")
        tutorial.go_to_pose_goal(0.3,0.4,0.45,0,np.pi,0)
        tutorial.go_to_pose_goal(0.3,0.4,0.35,0,np.pi,0)

        input("============ Press `Enter` to start the movement ...")
        tutorial.traj([0.52,0.40,0.275],[-1,0,0.25],0.2,0.35,50)
      
        node.get_logger().info("finito")
        
    except KeyboardInterrupt:
        node.get_logger().info("Arrêté par l'utilisateur (KeyboardInterrupt)")
    finally:
        # Nettoyage
        node.get_logger().info("Arrêt de MoveIt, de l'exécuteur et de rclpy...")
        moveit_commander.roscpp_shutdown()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        executor_thread.join()


if __name__ == "__main__":
    main()