import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
import sys

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "odometry2"
    pkg_share_path = get_package_share_directory(pkg_name)
    
    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))

    ld.add_action(launch.actions.DeclareLaunchArgument("debug", default_value="false"))
    dbg_sub = launch.substitutions.PythonExpression(['"" if "false" == "', launch.substitutions.LaunchConfiguration("debug"), '" else "debug_ros2launch ' + os.ttyname(sys.stdout.fileno()) + '"'])

    DRONE_DEVICE_ID=os.getenv('DRONE_DEVICE_ID')

    namespace=DRONE_DEVICE_ID
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
                    {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                ],
                remappings=[
                    ("~/local_odom_out", "~/local_odom"),
                    ("~/local_hector_out", "~/local_hector"),

                    ("~/timesync_in", "/" + DRONE_DEVICE_ID + "/Timesync_PubSubTopic"),
                    ("~/gps_in", "/" + DRONE_DEVICE_ID + "/VehicleGpsPosition_PubSubTopic"),
                    ("~/pixhawk_odom_in", "/" + DRONE_DEVICE_ID + "/VehicleOdometry_PubSubTopic"),
                    ("~/hector_pose_in", "/" + DRONE_DEVICE_ID + "/slam_out_pose"),
                    ("~/baro_in", "/" + DRONE_DEVICE_ID + "/SensorBaro_PubSubTopic"),
                    ("~/garmin_in", "/" + DRONE_DEVICE_ID + "/garmin/range"),
                    ("~/gps_raw_in", "/" + DRONE_DEVICE_ID + "/VehicleGpsPosition_PubSubTopic"),

                    ("~/set_origin_out", "/" + DRONE_DEVICE_ID + "/control_interface/set_origin"),
                    # ("~/sensor_gps_out", "/" + DRONE_DEVICE_ID + "/SensorGps_PubSubTopic"),
                    ("~/visual_odom_out", "/" + DRONE_DEVICE_ID + "/VehicleVisualOdometry_PubSubTopic"),

                    ("~/get_origin", "~/get_origin"),
                    ("~/getting_odom", "~/getting_odom"),
                ],
            ),
        ],
        output='screen',
        prefix=dbg_sub,
        parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
    ))

    return ld
