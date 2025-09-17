from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription()

    monocular_node = Node(
        package="orbslam3_ros2",
        executable="rgbd_inertial",
        namespace="", # MODIFY
        remappings=[
            ("rgb", "camera/down/color/image_raw"),
            ("depth", "camera/down/depth/image_rect_raw"),
            ("imu", "camera/down/imu"),
        ],
        parameters=[
            {"orb_voc_path": str(os.environ.get("ORB_VOC_PATH"))},
            {"settings_path": str(os.environ.get("SIMPLE_CAM_CONFIG_PATH"))},
            {"camera_fps": 30},
        ]
    )

    ld.add_action(monocular_node)

    return ld
