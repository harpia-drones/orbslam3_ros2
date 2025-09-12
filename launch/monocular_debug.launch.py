from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription()

    monocular_debug_node = Node(
        package="slam",
        executable="monocular_debug",
        namespace="",
        remappings=[
            ("color", "color/compressed"),
        ],
        parameters=[
            {"orb_voc_path": str(os.environ.get("ORB_VOC_PATH"))},
            {"settings_path": str(os.environ.get("TUM1_PATH"))},
            {"camera_fps": 30},
        ]
    )

    ld.add_action(monocular_debug_node)

    return ld