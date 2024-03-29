import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    TimerAction,
    OpaqueFunction,
    PushLaunchConfigurations,
    PopLaunchConfigurations,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    # Params
    cslam_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cslam_experiments"),
                "launch",
                "cslam",
                "cslam_rgbd.launch.py",
            )
        ),
        launch_arguments={
            "config_path": os.path.join(
                get_package_share_directory("cslam_experiments"), "config/"
            ),
            "config_file": LaunchConfiguration("cslam_config_file").perform(context),
            "robot_id": LaunchConfiguration("robot_id").perform(context),
            "namespace": "/r" + LaunchConfiguration("robot_id").perform(context),
            "max_nb_robots": LaunchConfiguration("max_nb_robots").perform(context),
        }.items(),
    )

    # Odom
    odom_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cslam_experiments"),
                "launch",
                "odometry",
                "rtabmap_d455i_odometry.launch.py",
            )
        ),
        launch_arguments={
            "namespace": "/r" + LaunchConfiguration("robot_id").perform(context),
            "robot_id": LaunchConfiguration("robot_id").perform(context),
            "log_level": "error",
        }.items(),
    )

    bag_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cslam_experiments"),
                "launch",
                "sensors",
                "bag_campus.launch.py",
            )
        ),
        launch_arguments={
            "namespace": "/r" + LaunchConfiguration("robot_id").perform(context),
            "bag_file": LaunchConfiguration("bag_file").perform(context),
            "rate": LaunchConfiguration("rate").perform(context),
            "robot_name": LaunchConfiguration("robot_name").perform(context),
            "start": LaunchConfiguration("start").perform(context),
            "duration": LaunchConfiguration("duration").perform(context),
        }.items(),
    )

    # Launch schedule
    schedule = []

    schedule.append(PushLaunchConfigurations())
    schedule.append(cslam_proc)
    schedule.append(PopLaunchConfigurations())

    schedule.append(PushLaunchConfigurations())
    schedule.append(odom_proc)
    schedule.append(PopLaunchConfigurations())

    schedule.append(PushLaunchConfigurations())
    schedule.append(bag_proc)
    schedule.append(PopLaunchConfigurations())

    return schedule


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="0"),
            DeclareLaunchArgument("max_nb_robots", default_value="5"),
            DeclareLaunchArgument(
                "cslam_config_file", default_value="realsense_rgbd.yaml", description=""
            ),
            DeclareLaunchArgument(
                "bag_file",
                default_value=os.path.join(
                    "/datasets/campus-x/outdoor/10_14_sparkal1",
                ),
                description="",
            ),
            DeclareLaunchArgument("rate", default_value="1.0", description="Bag rate"),
            DeclareLaunchArgument(
                "robot_name", default_value="sparkal1", description=""
            ),
            DeclareLaunchArgument(
                "start", default_value="0.0", description="Bag start time"
            ),
            DeclareLaunchArgument(
                "duration", default_value="0.0", description="Bag duration"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
