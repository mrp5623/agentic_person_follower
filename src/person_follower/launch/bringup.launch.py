from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('person_follower'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([

        Node(
            package='person_follower',
            executable='perception_node',
            name='perception_node',
            parameters=[config],
            output='screen'
        ),

        Node(
            package='person_follower',
            executable='tracker_node',
            name='tracker_node',
            parameters=[config],
            output='screen'
        ),

        Node(
            package='person_follower',
            executable='agent_node',
            name='agent_node',
            parameters=[config],
            output='screen'
        ),

        Node(
            package='person_follower',
            executable='controller_node',
            name='controller_node',
            parameters=[config],
            output='screen'
        ),

    ])