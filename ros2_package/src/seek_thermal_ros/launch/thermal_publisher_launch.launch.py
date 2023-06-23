from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='seek_thermal_ros',
            executable='thermal_publisher',
            name='thermal_publisher',
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='thermal_republish',
            arguments=[  
                'raw',
                'compressed'
            ],
            remappings=[
                ('in', '/thermalImage'),
                ('out','/thermalImage/compressed')
            ]
        ),           
    ])