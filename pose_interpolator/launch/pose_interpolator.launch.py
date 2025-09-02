from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pose_interpolator',
            executable='pose_interpolator_node',
            name='pose_interpolator',
            output='screen',
            parameters=[{
                'output_frequency': 50.0,
                'max_interpolation_time': 0.5
            }],
            remappings=[
                ('/tracked_pose', '/tracked_pose'),
                ('/odom', '/odom'),
                ('/global_pose', '/global_pose')
            ]
        )
    ])
