#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import copy
import math # Import nécessaire pour PI
from geometry_msgs.msg import Pose, Point, Quaternion

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
        self.target_pose = Pose(
        position=Point(x=0.3, y=-0.2, z=0.2),
        orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))

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
    # 3. COMMANDE DE POSE (Planning OMPL)
    # ---------------------------------------------------------
    def send_pose_goal(self, target_pose, tolerance=0.01):
        """
        Demande au planificateur (OMPL) de trouver un chemin vers une pose (Position + Orientation).
        Ce n'est PAS une ligne droite, le robot va contourner les obstacles si besoin.
        """
        self.movement_done = False
        self.success = False
        print(f"\n--- Envoi Commande Pose (IK Planning) ---")

        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.num_planning_attempts = 10

        # Création des contraintes
        constraints = Constraints()

        # --- A. Contrainte de Position ---
        # Pour MoveIt, une contrainte de position est un "Volume" (BoundingVolume)
        # dans lequel le lien (tool0) doit se trouver.
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tool0"
        pc.weight = 1.0
        
        # On définit une "boîte" de tolérance autour du point cible
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [tolerance, tolerance, tolerance] # x, y, z dimensions
        
        # La pose de cette boîte est la position cible
        box_pose = Pose()
        box_pose.position = target_pose.position
        box_pose.orientation.w = 1.0 # L'orientation de la boite elle-même n'importe pas

        bv = BoundingVolume()
        bv.primitives.append(box)
        bv.primitive_poses.append(box_pose)
        
        pc.constraint_region = bv
        constraints.position_constraints.append(pc)

        # --- B. Contrainte d'Orientation ---
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "tool0"
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = tolerance
        oc.absolute_y_axis_tolerance = tolerance
        oc.absolute_z_axis_tolerance = tolerance
        oc.weight = 1.0
        
        constraints.orientation_constraints.append(oc)

        # Ajout des contraintes à la requête
        goal_msg.request.goal_constraints.append(constraints)

        # Envoi
        self._send_goal_future = self._move_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

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

    def traj(self):
        waypoints = []
        w0 = copy.deepcopy(self.target_pose)
        waypoints.append(w0)
        for i in range(50):
            w_up = copy.deepcopy(waypoints[-1])
            w_up.position.z += 0.002
            w_up.position.y -= 0.002

            waypoints.append(w_up)  
        return waypoints
    
    def func(self, x):
        return x**2 / 0.3  
    
    def generate_math_path(self, func, x_start, x_end, num_steps=50):
        """
        Génère des waypoints basés sur une fonction mathématique z = f(x).
        Le mouvement se fera dans le plan X-Z du robot (avance et monte/descend).
        
        :param func: Une fonction lambda ou def, ex: lambda x: x**2
        :param x_start: Valeur x de départ (relative à la pose actuelle)
        :param x_end: Valeur x de fin
        :param num_steps: Nombre de points pour lisser la courbe
        """
        waypoints = []
        
        # On part de la position cible actuelle (ou self.target_pose)
        base_pose = copy.deepcopy(self.target_pose)
        
        # Calcul du pas
        step_size = (x_end - x_start) / num_steps

        for i in range(num_steps + 1):
            x_math = x_start + (i * step_size)
            
            z_math = func(x_math)
            
            w_pose = copy.deepcopy(base_pose)
            
            w_pose.position.x -= x_math
            w_pose.position.z += z_math
            
            waypoints.append(w_pose)
            
        return waypoints
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    ur3_client = UR3MoveItActionClient()
    
    print("Synchronisation joint_states...")
    while ur3_client._latest_joint_state is None:
        rclpy.spin_once(ur3_client, timeout_sec=0.1)

    
    try:
        
        ur3_client.send_pose_goal(ur3_client.target_pose)
        if not ur3_client.wait_for_completion():
            print("Échec du Pose Goal.")
            return
        time.sleep(1.0)

        # ---------------------------------------------------------
        # ETAPE 3 : Mouvement Linéaire (Cartesian Path)
        # ---------------------------------------------------------
        print("\n--- Préparation Trajectoire Linéaire ---")
        
        ur3_client.send_cartesian_path(ur3_client.generate_math_path(
            ur3_client.func, x_start=0.0, x_end=0.3))
        ur3_client.wait_for_completion()
        
        print("Script terminé avec succès.")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Erreur critique : {e}")
    finally:
        # Bonne pratique : s'assurer que ROS s'éteint proprement
        rclpy.shutdown()

if __name__ == '__main__':
    main()