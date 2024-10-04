from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Define the path to wit_ros2_imu.py in the wit_ros2_imu package
    wit_ros2_imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',  # Assuming wit_ros2_imu.py is made executable
        name='wit_ros2_imu_node',
        output='screen',
    )

    # Define the path to serialpid_v1.2.py in the balance package
    serialpid_node = Node(
        package='balance',
        executable='serialpid_v1_2',  # Assuming serialpid_v1.2.py is made executable
        name='serialpid_node',
        output='screen',
    )

    joy_controller_node = Node(
        package='wit_ros2_imu',
        executable='joy_controller_new',  # Assuming serialpid_v1.2.py is made executable
        name='joy_controller_node',
        output='screen',
    )

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        )

    return LaunchDescription([
        wit_ros2_imu_node,
        serialpid_node,
        joy_controller_node,
        joy_node
    ])

