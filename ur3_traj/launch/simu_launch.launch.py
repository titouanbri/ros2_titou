import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Récupérer le chemin du package ur_simulation_gazebo
    ur_sim_pkg_share = get_package_share_directory('ur_simulation_gazebo')

    # 2. Définir l'inclusion du fichier launch de simulation avec l'argument ur_type
    # Correspond à : ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur3
    ur_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_sim_pkg_share, 'launch', 'ur_sim_moveit.launch.py')
        ),
        launch_arguments={'ur_type': 'ur3'}.items()
    )

    # 3. Définir le nœud scene_object
    # Correspond à : ros2 run ur3_traj scene_object
    scene_object_node = Node(
        package='ur3_traj',
        executable='scene_object',
        name='scene_object',
        output='screen'
    )

    # 4. Retourner la description du launch
    return LaunchDescription([
        ur_simulation_launch,
        scene_object_node
    ])