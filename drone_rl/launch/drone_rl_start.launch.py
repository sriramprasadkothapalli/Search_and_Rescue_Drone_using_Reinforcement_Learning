"""
Demo for spawn_entity.
Launches Gazebo and spawns a model
"""
# A bunch of software packages that are needed to launch ROS2
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'hospital.world'
    pkg_dir = get_package_share_directory('hospital_robot_spawner')

    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')
    # os.environ["GAZEBO_RESOURCE_PATH"] = os.path.join(pkg_dir, 'worlds')

    # world = os.path.join(pkg_dir, 'worlds', world_file_name)
    world = '/home/sriram/drone_ws/src/Drone-RL-Control/sjtu_drone_description/worlds/house.world'
    launch_file_dir = os.path.join(pkg_dir, 'launch')

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    # spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
    #                    arguments=['-entity', 'demo', 'x', 'y', 'z'],
    #                    output='screen')
    spawn_entity = Node(package='drone_control',
                        executable='spawn_drone',
                        arguments=['SJTU-DroneEnv', 'demo', '1', '16.0', '0.0'],
                        output='screen')

    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])