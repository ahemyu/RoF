from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_pubsub_package',
            namespace='',
            executable='publisher_node',
            name='minimal_publisher'
        ),
        Node(
            package='my_pubsub_package',
            namespace='',
            executable='subscriber_node',
            name='minimal_subscriber'
        ),
        ])