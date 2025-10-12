from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription()

    rgbd_inertial_v2_node = Node(
        package="orbslam3_ros2",
        executable="rgbd_inertial_v2",
        remappings=[
            ("/rgb", "/camera/realsense_d435i/color/image_raw"),
            ("/depth", "/camera/realsense_d435i/depth/image_rect_raw"),
            ("/imu", "/camera/realsense_d435i/imu"),
        ],
        parameters=[
            {"orb_voc_path": str(os.environ.get("ORB_VOC_PATH"))},
            {"settings_path": str(os.environ.get("D435I_CONFIG_PATH"))},
            {"camera_fps": 30},
            {"visualization": False},
        ]
    )

    ld.add_action(rgbd_inertial_v2_node)

    return ld