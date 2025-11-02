import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    rover_sim_path = os.path.join(
        get_package_share_directory('rover_sim'))

    xacro_file = os.path.join(rover_sim_path,
                              'urdf',
                              'rover_gazebo.xacro.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}

    print(params)

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'default',
            '-string', doc.toxml(),
            '-name', 'rover_model',
            '-allow_renaming', 'true',
            '-z', '1.0'  
        ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',

        ],
        remappings=[

        ],
        output='screen'
    )

    # World file path
    world_file = os.path.join(rover_sim_path, 'worlds', 'ground_plane.sdf')
    
    # Launch Gazebo Sim with ground plane world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                          'launch', 'gz_sim.launch.py')]),
        launch_arguments=[('gz_args', f'{world_file} -r -v 4')]  # -r for run, -v for verbosity
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock'),
        bridge,
        gazebo_launch,
        node_robot_state_publisher,
        ignition_spawn_entity,
    ])
