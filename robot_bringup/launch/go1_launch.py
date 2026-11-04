"""Bringup for the Unitree Go1 with hand gesture control.

Launches:
    - Go1 driver unitree_ros (optional, `use_driver:=false` to disable, `wifi:=true` for Wi-Fi)
    - RealSense camera driver (optional, `use_realsense:=false` to disable)
    - robot_bt behaviour tree server (go1_bt)
    - hand_gestures plugin (detector + annotator)
    - tello_control_station (pygame UI used to switch control modes)

Disable the driver (`use_driver:=false`) if it already runs elsewhere on the
same ROS 2 network (e.g. on the robot or in another container).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_DIR = get_package_share_directory("robot_bringup")
PARAMS_FILE = os.path.join(PKG_DIR, "config", "go1_params.yaml")


def create_go1_driver_launch(ld: LaunchDescription) -> None:
    ld.add_action(
        DeclareLaunchArgument(
            "use_driver",
            default_value="true",
            description="Start the Go1 driver (unitree_ros)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "wifi",
            default_value="false",
            description="Connect to the Go1 over Wi-Fi (192.168.12.1) instead of Ethernet (192.168.123.161)",
        )
    )
    unitree_pkg_dir = get_package_share_directory("unitree_ros")
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(unitree_pkg_dir, "launch", "unitree_driver_launch.py")
            ),
            launch_arguments={"wifi": LaunchConfiguration("wifi")}.items(),
            condition=IfCondition(LaunchConfiguration("use_driver")),
        )
    )


def create_realsense_launch(ld: LaunchDescription) -> None:
    ld.add_action(
        DeclareLaunchArgument(
            "use_realsense",
            default_value="true",
            description="Start the RealSense driver (publishes /camera/camera/color/image_raw)",
        )
    )
    realsense_pkg_dir = get_package_share_directory("realsense2_camera")
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_pkg_dir, "launch", "rs_launch.py")
            ),
            launch_arguments={
                "enable_depth": "false",
                "rgb_camera.color_profile": "640,480,30",
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_realsense")),
        )
    )


def create_robot_bt_launch(ld: LaunchDescription) -> None:
    robot_bt_pkg_dir = get_package_share_directory("robot_bt")
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bt_pkg_dir, "launch/robot_bt_launch.py")
            ),
            launch_arguments={"params_file": PARAMS_FILE}.items(),
        )
    )


def create_hand_gestures_plugin_launch(ld: LaunchDescription) -> None:
    hand_gestures_pkg_dir = get_package_share_directory("hand_gestures")
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hand_gestures_pkg_dir, "launch/hand_gestures_launch.py")
            ),
            launch_arguments={
                "params_file": PARAMS_FILE,
                "run_annotator": "true",
                "use_compression": "false",
            }.items(),
        )
    )


def create_control_station_launch(ld: LaunchDescription) -> None:
    ld.add_action(
        DeclareLaunchArgument(
            "use_control_station",
            default_value="true",
            description="Start the pygame control station (needs a display)",
        )
    )
    ld.add_action(
        Node(
            package="tello_control_station",
            executable="control_station",
            parameters=[PARAMS_FILE],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_control_station")),
        )
    )


def generate_launch_description():
    ld = LaunchDescription()

    create_go1_driver_launch(ld)
    create_realsense_launch(ld)
    create_robot_bt_launch(ld)
    create_hand_gestures_plugin_launch(ld)
    create_control_station_launch(ld)

    return ld
