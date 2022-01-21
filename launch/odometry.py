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

    dbg_sub = None
    if sys.stdout.isatty():
        dbg_sub = launch.substitutions.PythonExpression(['"" if "false" == "', launch.substitutions.LaunchConfiguration("debug"), '" else "debug_ros2launch ' + os.ttyname(sys.stdout.fileno()) + '"'])

    DRONE_DEVICE_ID=os.getenv('DRONE_DEVICE_ID')

    namespace=DRONE_DEVICE_ID
    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_odometry',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='odometry2::Odometry2',
                namespace=namespace,
                name='odometry',
                parameters=[
                    pkg_share_path + '/config/param.yaml',
                    {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                ],
                remappings=[
                    # publishers
                    ("~/local_odom_out", "~/local_odom"),
                    ("~/home_position_out", "~/home_position"),
                    # subscribers
                    ("~/pixhawk_odom_in", "/" + DRONE_DEVICE_ID + "/fmu/vehicle_odometry/out"),
                    ("~/home_position_in", "/" + DRONE_DEVICE_ID + "/fmu/home_position/out"),
                    ("~/control_interface_diagnostics_in", "/" + DRONE_DEVICE_ID + "/control_interface/diagnostics"),
                    # service_clients
                    ("~/set_px4_param_int", "/" + DRONE_DEVICE_ID + "/control_interface/set_px4_param_int"),
                    ("~/set_px4_param_float", "/" + DRONE_DEVICE_ID + "/control_interface/set_px4_param_float"),
                ],
            ),
        ],
        output='screen',
        prefix=dbg_sub,
        parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
    ))

    return ld
