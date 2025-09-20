import time
import unittest
import os
import sys

import launch
import launch_ros
import launch_testing.actions
import launch_testing.asserts
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
import rclpy
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
        LaunchConfiguration,
        PathJoinSubstitution,
        )

from std_msgs.msg import Bool

def generate_test_description():

    SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')

    ld = launch.LaunchDescription()

    uav_type="x500"
    uav_name="uav1"
    platform_config=get_package_share_directory("mrs_multirotor_simulator")+"/config/mrs_uav_system/"+uav_type+".yaml"

    launch_file_path = os.path.abspath(__file__)
    launch_dir = os.path.dirname(launch_file_path)

    test_name = os.path.basename(launch_dir)

    modality = LaunchConfiguration('modality')

    ld.add_action(DeclareLaunchArgument(
        'modality',
        default_value="default",
        description="",
        ))

    # ld.add_action(
    #         launch_ros.actions.Node(
    #             package='rmw_zenoh_cpp',
    #             namespace='',
    #             executable='rmw_zenohd',
    #             name='zenoh_router',
    #         )
    #     )

    ld.add_action(
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('mrs_uav_testing'),
                        'launch',
                        'mrs_uav_system.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'run_automatic_start': "true",
                        'uav_name': uav_name,
                        'platform_config': platform_config,
                        # 'world_config': launch_dir+"/config/world_config.yaml",
                        # 'custom_config': launch_dir+"/config/custom_config.yaml",
                        # 'automatic_start_config': launch_dir+"/config/automatic_start.yaml",
                    }.items()
                )
            ]
        )
    )

    ld.add_action(
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('mrs_uav_testing'),
                            'launch',
                            'mrs_multirotor_simulator.launch.py'
                        ])
                    ]),
                    # launch_arguments={
                    #     'custom_config': launch_dir+"/config/mrs_simulator.yaml",
                    # }.items()
                )
            ]
        )
    )

    ld.add_action(
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('mrs_multirotor_simulator'),
                            'launch',
                            'hw_api.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'custom_config': [launch_dir,"/config/hw_api/hw_api_",modality,".yaml"],
                    }.items()
                )
            ]
        )
    )

    # starts the integration interactor
    ld.add_action(
            # Nodes under test
            launch_ros.actions.Node(
                package='mrs_uav_controllers',
                namespace='',
                executable='test_'+test_name,
                name='test_'+test_name,
                output="screen",
                parameters=[
                        {'test_name': [test_name,"_",modality]},
                        {'modality': [modality]},
                ],
            )
        )

    # starts the python test part down below
    ld.add_action(
        launch.actions.TimerAction(
            period=1.0, actions=[launch_testing.actions.ReadyToTest()]),
        )

    return ld

# #{ class PublisherHandlerTest(unittest.TestCase)

class PublisherHandlerTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('integration_test_handler')

    def tearDown(self):
        self.node.destroy_node()

    def test_interactor(self, proc_output, timeout=120):

        """Check whether pose messages published"""

        test_result = []

        sub = self.node.create_subscription(
                Bool, '/test_result',
                lambda msg: test_result.append(msg), 100)
        try:

            end_time = time.time() + timeout

            while time.time() < end_time:

                if len(test_result) > 0:
                    break

                rclpy.spin_once(self.node, timeout_sec=1)

            time.sleep(2.0)

            # check if we have the result
            self.assertTrue(len(test_result) > 0)

            # check if the result is true
            self.assertTrue(test_result[0].data)

        finally:
            self.node.destroy_subscription(sub)

# #} end of

# #{ Post-shutdown tests

# @launch_testing.post_shutdown_test()
# class PublisherHandlerTestShutdown(unittest.TestCase):
#     def test_exit_codes(self, proc_info):
#         """Check if the processes exited normally."""
#         launch_testing.asserts.assertExitCodes(proc_info)

# #} end of Post-shutdown tests
