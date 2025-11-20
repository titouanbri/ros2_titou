#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import copy
import math # Import nécessaire pour PI

# Messages ROS2 / MoveIt
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume, RobotState
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

class UR3MoveItActionClient(Node):
    def __init__(self):
        super().__init__('ur3_moveit_client')
        
        self._move_action_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_action_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._compute_cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self._latest_joint_state = None

        # Gestion d'état
        self.movement_done = False
        self.success = False

        print("Attente des serveurs MoveIt...")
        self._move_action_client.wait_for_server()
        self._execute_action_client.wait_for_server()
        self._compute_cartesian_client.wait_for_service()
        print("Connecté !")

    def joint_state_callback(self, msg):
        self._latest_joint_state = msg

    def wait_for_completion(self):
        """Attend la fin de l'action en cours"""
        print("   -> Mouvement en cours...")
        while not self.movement_done:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.success

    # ---------------------------------------------------------
    # 1. COMMANDE ARTICULAIRE (La méthode "inratable")
    # ---------------------------------------------------------
    def send_joint_goal(self, joint_values):
        self.movement_done = False
        self.success = False
        print(f"\n--- Envoi Commande Articulaire (Joints) ---")
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.allowed_planning_time = 5.0
        
        # Ordre des joints pour UR : Pan, Lift, Elbow, Wrist1, Wrist2, Wrist3
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        
        constraints = Constraints()
        for i, name in enumerate(joint_names):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = joint_values[i]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraints)
        
        self._send_goal_future = self._move_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # ---------------------------------------------------------
    # 2. TRAJECTOIRE CARTÉSIENNE
    # ---------------------------------------------------------
    def send_cartesian_path(self, waypoints):
        self.movement_done = False
        self.success = False
        
        if self._latest_joint_state is None:
            self.get_logger().error("Pas de joint_states !")
            self.movement_done = True
            return

        print(f"\n--- Calcul Trajectoire Cartésienne ({len(waypoints)} pts) ---")
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.group_name = "ur_manipulator"
        req.link_name = "tool0"
        req.waypoints = waypoints
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.start_state = RobotState()
        req.start_state.joint_state = self._latest_joint_state

        future = self._compute_cartesian_client.call_async(req)
        future.add_done_callback(self._cb_cartesian_computed)

    def _cb_cartesian_computed(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service failed: {e}')
            self.movement_done = True
            return

        if response.fraction < 0.90:
            self.get_logger().warn(f"Trajectoire incomplète ({response.fraction*100:.1f}%). Abandon.")
            self.movement_done = True
            return

        print("Trajectoire valide. Exécution...")
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        self._send_exec_future = self._execute_action_client.send_goal_async(goal_msg)
        self._send_exec_future.add_done_callback(self.goal_response_callback)

    # ---------------------------------------------------------
    # CALLBACKS
    # ---------------------------------------------------------
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Ordre rejeté par le serveur.")
            self.movement_done = True
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            print(">>> SUCCÈS !")
            self.success = True
        else:
            print(f">>> ÉCHEC. Code: {result.error_code.val}")
            self.success = False
        self.movement_done = True


def main(args=None):
    rclpy.init(args=args)
    ur3_client = UR3MoveItActionClient()
    
    print("Synchronisation joint_states...")
    while ur3_client._latest_joint_state is None:
        rclpy.spin_once(ur3_client, timeout_sec=0.1)

    # --- A. CONFIGURATION DU POINT DE DÉPART (EN ANGLES) ---
    # Ces angles placent le robot dans une position sûre pour démarrer (tête en bas)
    # Pan, Lift, Elbow, Wrist1, Wrist2, Wrist3 (en radians)
    start_joints = [0.0, -1.57, -1.57, -1.57, 1.57, 0.0]

    # --- B. DÉFINITION DE LA TRAJECTOIRE CARTÉSIENNE ---
    # Note : Le premier point DOIT être proche de la position atteinte par start_joints
    # Avec start_joints ci-dessus, on arrive environ à : x=0.1, y=0.3, z=0.3 (à vérifier selon le robot)
    
    # On définit une orientation "tête en bas" (Standard UR : rotation 180 autour de Y)
    # w=0, x=0, y=1, z=0
    
    p1 = Pose()
    p1.position.x = 0.2
    p1.position.y = 0.2
    p1.position.z = 0.3
    p1.orientation.w = 0.0; p1.orientation.x = 0.0; p1.orientation.y = 1.0; p1.orientation.z = 0.0
    
    waypoints = []
    waypoints.append(p1) 
    
    p2 = copy.deepcopy(p1); p2.position.y -= 0.15; waypoints.append(p2)
    p3 = copy.deepcopy(p2); p3.position.z -= 0.10; waypoints.append(p3)
    p4 = copy.deepcopy(p3); p4.position.y += 0.10; waypoints.append(p4)
    
    try:
        # 1. D'ABORD : On bouge en articulaires (Joints)
        # C'est beaucoup plus sûr que "send_pose_goal" pour la première approche
        ur3_client.send_joint_goal(start_joints)
        if not ur3_client.wait_for_completion():
            print("Échec de la mise en place joint.")
            return 

        time.sleep(0.5)

        # 2. ENSUITE : On lance la trajectoire carrée
        # Note : Comme on a bougé en joints, on n'est pas *exactement* sur P1 au début.
        # MoveIt va calculer le chemin de "Là où on est" -> "P1" -> "P2"...
        ur3_client.send_cartesian_path(waypoints)
        ur3_client.wait_for_completion()
        
        print("Script terminé avec succès.")
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

    rclpy.shutdown()

if __name__ == '__main__':
    main()