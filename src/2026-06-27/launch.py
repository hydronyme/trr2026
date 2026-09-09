from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/course_v1/brain_status'
        ],
        output='screen'
    )

    robot_node = [
        Node(
            package='course_v1',
            namespace='course_v1',
            executable='camera',
        ),
        Node(
            package='course_v1',
            namespace='course_v1',
            executable='web',
        ),
        Node(
            package='course_v1',
            namespace='course_v1',
            executable='brain',
        )
    ]

    return LaunchDescription(
        robot_node + [bag_record]
    )
