import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Get the model to load
    if 'SUB_MODEl' in os.environ:
        model = os.environ['SUB_MODEL']
    else:
        model = 'sub_imu'
    print('Using model: ' + model)

    # Get the launch directory
    br_description_dir = get_package_share_directory('sub_descriptions')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    namespace = DeclareLaunchArgument('namespace', default_value='sub', description='node namespace of sub')
    x_arg = DeclareLaunchArgument('x', default_value=0., description='x coordinate of sub')
    y_arg = DeclareLaunchArgument('y', default_value=0., description='y coordinate of sub')
    z_arg = DeclareLaunchArgument('z', default_value=0., description='z coordinate of sub')

    xacro_file = os.path.join(br_description_dir, 'robots', model + '.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')
    URDF_FILE = '/tmp/sub.urdf'
    with open(URDF_FILE, 'w') as f:
        f.write(robot_desc)
    params = {'use_sim_time': use_sim_time}

    upload_gazebo_node = Node(
        package='gazebo_ros',
        node_executable='spawn_entity.py',
        node_name='urdf_spawner',
        output='screen',
        arguments=[f"-gazebo_namespace /gazebo -x {LaunchConfiguration('x')} -y {LaunchConfiguration('x')} -z {LaunchConfiguration('x')} -entity {LaunchConfiguration('namespace')} -file {URDF_FILE}"]
    )

    run_state_publisher_node = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        output='screen',
        parameters=[params],
        arguments=[URDF_FILE]
    )

    # Create the launch description and populate
    ld = LaunchDescription([upload_gazebo_node, run_state_publisher_node, declare_sim_time_cmd, namespace, x_arg, y_arg, z_arg])

    return ld
