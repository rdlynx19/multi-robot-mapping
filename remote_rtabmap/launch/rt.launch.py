import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration("name").perform(context)
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    params_file = LaunchConfiguration("params_file")
    parameters = [
        {
            "frame_id": name,
            "subscribe_rgb": True,
            "subscribe_depth": True,
            "subscribe_odom_info": True,
            "wait_imu_to_init": True,
            "approx_sync": True,
            "Rtabmap/DetectionRate": "3.5",
        }
    ]

    remappings = [
        ("rgb/image", name + "/rgb/image_rect"),
        ("rgb/camera_info", name + "/rgb/camera_info"),
        ("depth/image", name + "/stereo/image_raw"),
        ("imu", name + "/imu/data"),
    ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "camera.launch.py")
            ),
            launch_arguments={"name": name, "params_file": params_file}.items(),
        ),
        LoadComposableNodes(
            target_container=name + "_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="rtabmap_odom",
                    plugin="rtabmap_odom::RGBDOdometry",
                    name="rgbd_odometry",
                    parameters=parameters,
                    remappings=remappings,
                ),
            ],
        ),
        LoadComposableNodes(
            target_container=name + "_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="rtabmap_slam",
                    plugin="rtabmap_slam::CoreWrapper",
                    name="rtabmap",
                    parameters=parameters,
                    remappings=remappings,
                ),
            ],
        ),
        Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            output="screen",
            parameters=parameters,
            remappings=remappings,
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "rgbd.yaml"),
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
