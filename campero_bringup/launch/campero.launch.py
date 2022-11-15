from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter, PushRosNamespace

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import yaml


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    joystick_type = LaunchConfiguration("joystick_type").perform(context)
    launch_gazebo = LaunchConfiguration("launch_gazebo").perform(context)
    urdf_description = LaunchConfiguration("urdf_description").perform(context)

    if robot_namespace:
        robot_description_name = "/" + robot_namespace + "/robot_description"
        controller_manager_name = "/" + robot_namespace + "/controller_manager"
        joints_prefix = robot_namespace + "_"
    else:
        robot_description_name = "/robot_description"
        controller_manager_name = "/controller_manager"
        joints_prefix = ""

    if robot_model == "rubber":
        kinematic_type = "skid_steering"
        command_message_type = "romea_mobile_base_msgs/SkidSteeringCommand"
    else:
        kinematic_type = "omni_steering"
        command_message_type = "romea_mobile_base_msgs/OmniSteeringCommand"

    launch_gazebo = (mode == "simulation") and launch_gazebo
    use_sim_time = (mode == "simulation") or (mode == "replay")

    base_description_yaml_file = (
        get_package_share_directory("campero_description")
        + "/config/campero_"
        + robot_model
        + ".yaml"
    )

    joystick_remapping_yaml_file = (
        get_package_share_directory("romea_teleop")
        + "/config/"
        + joystick_type
        + "_"
        + kinematic_type
        + "_remappings.yaml"
    )

    controller_manager_yaml_file = (
        get_package_share_directory("campero_bringup")
        + "/config/controller_manager.yaml"
    )

    base_controller_yaml_file = (
        get_package_share_directory("campero_bringup")
        + "/config/mobile_base_controller.yaml"
    )

    command_message_priority = 100

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
                )
            ]
        ),
        launch_arguments={"verbose": "false"}.items(),
        condition=IfCondition(str(launch_gazebo)),
    )

    robot_description = {"robot_description": urdf_description}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_entity = Node(
        condition=LaunchConfigurationEquals("mode", "simulation"),
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            robot_description_name,
            "-entity",
            robot_namespace,
            "-robot_namespace",
            robot_namespace,
        ],
        output="screen",
    )

    campero_bridge = Node(
        condition=LaunchConfigurationEquals("mode", "live"),
        package="campero_bridge",
        executable="campero_bridge",
        output="screen",
    )

    controller_manager = Node(
        condition=LaunchConfigurationEquals("mode", "live"),
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_manager_yaml_file],
        output="screen",
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_mobile_base_controllers"),
                        "launch",
                        "mobile_base_controller.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "joints_prefix": joints_prefix,
            "controller_name": "mobile_base_controller_" + robot_model,
            "controller_manager_name": controller_manager_name,
            "base_description_yaml_filename": base_description_yaml_file,
            "base_controller_yaml_filename": base_controller_yaml_file,
        }.items(),
        condition=LaunchConfigurationNotEquals("mode", "replay"),
    )

    joy = Node(
        condition=LaunchConfigurationNotEquals("mode", "replay"),
        package="joy",
        executable="joy_node",
        name="joy",
        output="log",
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_teleop"),
                        "launch",
                        kinematic_type + "_teleop.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "joystick_type": joystick_type,
            "output_message_type": command_message_type,
            "output_message_priority": str(command_message_priority),
            "base_description_yaml_filename": base_description_yaml_file,
        }.items(),
    )

    cmd_mux = Node(
        condition=LaunchConfigurationNotEquals("mode", "replay"),
        package="romea_cmd_mux",
        executable="cmd_mux_node",
        name="cmd_mux",
        parameters=[{"topics_type": command_message_type}],
        remappings=[("~/out", "controller/cmd_" + kinematic_type)],
        output="screen",
    )

    return [
        gazebo,
        campero_bridge,
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=use_sim_time),
                PushRosNamespace(robot_namespace),
                robot_state_publisher,
                spawn_entity,
                controller_manager,
                controller,
                joy,
                teleop,
                cmd_mux,
            ]
        ),
    ]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_model"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("joystick_type"))

    declared_arguments.append(DeclareLaunchArgument("launch_gazebo"))

    declared_arguments.append(DeclareLaunchArgument("urdf_description"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
