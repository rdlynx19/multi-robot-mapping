import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    parameters=[{'frame_id':'oak-d-base-frame',
                 'subscribe_rgbd': True,
                 'approx_sync': False,
                 'wait_imu_to_init': True,
                 'topic_queue_size': 20,
                 'sync_queue_size': 20}]

    remappings = [('imu', '/imu/data')]

    return LaunchDescription([
        #Launch rgbd camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('depthai_examples'), 'launch'),
                '/rgb_stereo_node.launch.py']),
                launch_arguments={'depth_aligned':'false'},
                                  'enableRviz':'false',
                                  'monoResolution':'400p'}.items(),
        ),
                            
