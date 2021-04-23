import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "odometry2"
    pkg_share_path = get_package_share_directory(pkg_name)
    
    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))

    UAV_TYPE=os.getenv('UAV_TYPE')
    UAV_NAME=os.getenv('UAV_NAME')

    namespace=UAV_NAME
    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_odometry2',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='odometry2::Odometry2',
                namespace=namespace,
                name='odometry2',
                parameters=[
                    pkg_share_path + '/config/odometry2.yaml',
                    {"uav_type": UAV_TYPE},
                    {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                ],
                remappings=[
                    ("~/vehicle_command_out", "/VehicleCommand_PubSubTopic"),
                    ("~/local_odom_out", "~/local_odom"),
                    ("~/debug_markers_out", "~/debug/markers"),

                    ("~/timesync_in", "/Timesync_PubSubTopic"),
                    ("~/gps_in", "/VehicleGlobalPosition_PubSubTopic"),
                    ("~/pixhawk_odom_in", "/VehicleOdometry_PubSubTopic"),
                    ("~/waypoints_in", "/waypoints"),

                    ("~/offboard_control_mode_out", "/OffboardControlMode_PubSubTopic"),
                    ("~/position_setpoint_triplet_out", "/PositionSetpointTriplet_PubSubTopic"),
                    ("~/takeoff_in", "/takeoff"),
                    ("~/land_in", "/land"),
                    ("~/land_home_in", "/land_home"),
                    ("~/local_setpoint_in", "/local_setpoint"),
                ],
            ),
        ],
        output='screen',
        parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
    ))

    return ld
