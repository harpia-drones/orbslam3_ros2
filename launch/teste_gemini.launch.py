from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription()

    teste_gemini_node = Node(
        package="orbslam3_ros2",
        executable="teste_gemini",
        remappings=[
            ("/rgb", "/camera/front/color/image_raw"),
            ("/depth", "/camera/front/aligned_depth_to_color/image_raw"),
            ("/imu", "/camera/front/imu"),
        ],
        parameters=[
            {"orb_voc_path": str(os.environ.get("ORB_VOC_PATH"))},
            {"settings_path": str(os.environ.get("D435I_CONFIG_PATH"))},
            {"camera_fps": 30},
            {"visualization": False},
        ]
    )

    ld.add_action(teste_gemini_node)

    return ld


# ~/harpia_ws/install/orbslam3_ros2/lib/orbslam3_ros2/teste_gemini --ros-args -p orb_voc_path:=$ORB_VOC_PATH -p settings_path:=$D435I_CONFIG_PATH -p camera_fps:=30 -p visualization:=false -r /rgb:=/camera/front/color/image_raw -r /depth:=/camera/front/aligned_depth_to_color/image_raw -r /imu:=/camera/front/imu
