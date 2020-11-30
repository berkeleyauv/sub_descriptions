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

    xacro_file = os.path.join(br_description_dir, 'robots', model + '.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent='  ')
    with open('/tmp/sub.urdf', 'x') as f:
        f.write(robot_desc)
    params = {'use_sim_time': use_sim_time}

    run_state_publisher_node = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        parameters=[params],
        arguments=['/tmp/sub.urdf'])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_sim_time_cmd)
    ld.add_action(run_state_publisher_node)

    return ld
