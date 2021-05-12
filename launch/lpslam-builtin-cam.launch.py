from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='v4l2_camera',
        #     namespace='lpslam',
        #     executable='v4l2_camera_node',
        #     name='lpslam_camera',
        #     parameters=[
        #         #{"image_size" : [2560,720]}
        #         {"image_size" : [1344,376]}
        #     ]
        # ),
        # Node(
        #     package='rqt_image_view',
        #     executable='rqt_image_view',
        #     name='rqt_image_view'
        # ),
        Node(
            package='lpslam',
            namespace='lpslam',
            executable='lpslam_node',
            name='lpslam_compute',
            remappings=[
                ('image_raw', '/lpslam/image_raw')
            ],
            parameters=[
                {"config_file": "/home/icarrymini/icarryminiros2/lpslam_ros2/lpslam/configs/zed2-stereocam-openvslam-1280-SN25661426.json"}
            ]
        )
    ])
