import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    PushLaunchConfigurations,
    PopLaunchConfigurations,
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    rate = float(LaunchConfiguration("rate").perform(context))
    bag_start_delay = float(LaunchConfiguration("bag_start_delay").perform(context))
    bag_start_delay /= rate

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("cslam_experiments"),
                    "launch",
                    "sensors",
                    "campus_tf.launch.py",
                )
            ),
            launch_arguments={
                "base_link": LaunchConfiguration("namespace").perform(context) + "_link"
            }.items(),
        ),
        PushLaunchConfigurations(),
        Node(
            package="image_transport",
            executable="republish",
            name="republish",
            arguments=[
                "compressed",
                "raw",
                "--ros-args",
                "-r",
                "in/compressed:="
                + LaunchConfiguration("namespace").perform(context)
                + "/color/image_raw/compressed",
                "-r",
                "out:="
                + LaunchConfiguration("namespace").perform(context)
                + "/color/image_raw",
            ],
        ),
        PopLaunchConfigurations(),
        TimerAction(
            period=bag_start_delay,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "bag",
                        "play",
                        LaunchConfiguration("bag_file").perform(context),
                        "-r",
                        LaunchConfiguration("rate"),
                        "--remap",
                        # fmt: off
                        "/"+LaunchConfiguration("robot_name").perform(context)+"/forward/color/image_raw/compressed:=" + LaunchConfiguration("namespace").perform(context)+"/color/image_raw/compressed",
                        "/"+LaunchConfiguration("robot_name").perform(context)+"/forward/color/camera_info:=" + LaunchConfiguration("namespace").perform(context) + "/color/camera_info",
                        "/"+LaunchConfiguration("robot_name").perform(context)+"/forward/depth/image_rect_raw:=" + LaunchConfiguration("namespace").perform(context) + "/aligned_depth_to_color/image_raw",
                        "/"+LaunchConfiguration("robot_name").perform(context)+"/forward/imu:=" + LaunchConfiguration("namespace").perform(context) + "/"+LaunchConfiguration("robot_name").perform(context) + "/forward/imu",
                        # fmt: on
                    ],
                    name="bag",
                    output="screen",
                )
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag_file", default_value="", description=""),
            DeclareLaunchArgument("namespace", default_value="/r0", description=""),
            DeclareLaunchArgument("rate", default_value="1.0", description=""),
            DeclareLaunchArgument(
                "bag_start_delay", default_value="5.0", description=""
            ),
            DeclareLaunchArgument(
                "robot_name", default_value="sparkal1", description=""
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
