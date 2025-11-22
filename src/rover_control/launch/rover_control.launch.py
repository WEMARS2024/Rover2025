from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
  pkg_share = get_package_share_directory('rover_control')

  xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'rover_ros2_control.xacro'])
  controllers_file = PathJoinSubstitution([pkg_share, 'config', 'rover_controllers.yaml'])

  can_interface = LaunchConfiguration('can_interface')
  front_left_can_id = LaunchConfiguration('front_left_can_id')
  front_right_can_id = LaunchConfiguration('front_right_can_id')
  rear_left_can_id = LaunchConfiguration('rear_left_can_id')
  rear_right_can_id = LaunchConfiguration('rear_right_can_id')

  robot_description = ParameterValue(
    Command([
      'xacro ', xacro_file,
      ' can_interface:=', can_interface,
      ' front_left_can_id:=', front_left_can_id,
      ' front_right_can_id:=', front_right_can_id,
      ' rear_left_can_id:=', rear_left_can_id,
      ' rear_right_can_id:=', rear_right_can_id,
    ]),
    value_type=str)

  controller_manager = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[{'robot_description': robot_description}, controllers_file],
    output='screen',
  )

  joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster'],
    output='screen',
  )

  diff_drive_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['rover_base_controller', '--controller-manager-timeout', '120'],
    output='screen',
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'can_interface',
      default_value='can0',
      description='CAN interface used to communicate with the motor controllers'),
    DeclareLaunchArgument(
      'front_left_can_id',
      default_value='1',
      description='CAN node id for the front left wheel'),
    DeclareLaunchArgument(
      'front_right_can_id',
      default_value='2',
      description='CAN node id for the front right wheel'),
    DeclareLaunchArgument(
      'rear_left_can_id',
      default_value='3',
      description='CAN node id for the rear left wheel'),
    DeclareLaunchArgument(
      'rear_right_can_id',
      default_value='4',
      description='CAN node id for the rear right wheel'),
    controller_manager,
    joint_state_broadcaster_spawner,
    diff_drive_spawner,
  ])
