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
            'max_interpolation_time': 0.5,
            'receive_qr_time': 2.0,
            'odom_stop_time': 2.0,
            'limit_range_between_qr_cart': 0.1,
            'relocate_freeze_time' :20.0,
            'speed_threshold': 0.01,
            'qr_timeout_sec': 2.0,
            'publish_map_qr_tf': False,
            # 'use_sim_time': True
        }],
        remappings=[
            ('/tracked_pose', '/tracked_pose'),
            ('/odom', '/odom_combined'),
            ('/global_pose', '/global_pose')
        ]
    )
    ])
