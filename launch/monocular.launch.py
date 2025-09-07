from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = LaunchDescription()

    monocular_node = Node(
        package="slam",
        executable="monocular",
        namespace="", # MODIFY
        remappings=[
            ("color", "color/compressed"), # MODIFY
        ],
        parameters=[
            {"orb_voc_path": str(os.environ.get("ORB_VOC_PATH"))},
            {"settings_path": str(os.environ.get("SIMPLE_CAM_CONFIG_PATH"))},
            {"camera_fps": 30},
        ]
    )

    ld.add_action(monocular_node)

    return ld