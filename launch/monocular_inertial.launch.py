from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription()

    monocular_inertial_node = Node(
        package="orbslam3_ros2",
        executable="monocular_inertial",
        remappings=[
            ("/rgb", "/camera/down/color/image_raw"),
            ("/imu", "/camera/down/imu"),
        ],
        parameters=[
            {"orb_voc_path": str(os.environ.get("ORB_VOC_PATH"))},
            {"settings_path": str(os.environ.get("D435I_CONFIG_PATH"))},
            {"camera_fps": 30},
            {"visualization": False},
        ]
    )

    ld.add_action(monocular_inertial_node)

    return ld