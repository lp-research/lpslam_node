from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        Node(
            package='lpslam',
            #namespace='lpslam',
            executable='lpslam_node',
            name='lpslam_compute',
            parameters=[
                {"use_sim_time" : True},
                {"lpslam_config": "/home/poseidon/dev/lpr/icarryminiros2/lpslam_ros2/lpslam/configs/gazebo-openvslam.json"},
                {"write_lpslam_log" : True}
            ],
            # to run with gdb !
            #prefix=['xterm -e gdb -ex run --args'],
            arguments= [
                '--ros-args', '--log-level', 'INFO'
            ],
            remappings=[
                ('left_image_raw', '/camera_left/image_raw'),
                ('right_image_raw', '/camera_right/image_raw')
            ]
        )
    ])
