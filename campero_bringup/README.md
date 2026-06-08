# campero_bringup

## 1) Overview

`campero_bringup` connects the Campero description, hardware, bridge, simulation and teleoperation packages to the generic `romea_mobile_base_meta_bringup` workflow.

It provides:

* robot-specific generation functions for configuration, URDF and `ros2_control` descriptions;
* launch files for live control, simulation and teleoperation;
* controller manager and mobile base controller parameter files.

The supported Campero variants are `rubber` and `mecanum`.

## 2) Generated artifacts

The Python module `campero_bringup` delegates most generation work to `campero_description` and adds bringup-specific configuration such as the controller manager parameter file.

It provides the functions expected by `romea_mobile_base_meta_bringup`:

| Function | Purpose |
|---|---|
| `get_configuration(robot_model)` | returns the compact mobile base configuration for `rubber` or `mecanum` |
| `generate_configuration_file(robot_model, extended)` | generates the mobile base configuration file |
| `generate_urdf_description(prefix, mode, base_name, robot_model, ros_prefix)` | generates the Campero URDF description |
| `generate_ros2_control_description(prefix, mode, base_name, robot_model)` | generates the Campero `ros2_control` description |

The executable scripts in `scripts/` expose these functions from the command line.

## 3) Launch files

`launch/campero_base.launch.py` starts the Campero mobile base control stack. It starts the Campero bridge in live mode, publishes the complete `ros2_control` description, starts `controller_manager` outside Gazebo modes, loads the architecture-specific mobile base controller and starts `romea_cmd_mux`.

`launch/campero_teleop.launch.py` starts the mobile base teleoperation stack through `romea_mobile_base_teleop`. It selects the Campero mobile base configuration, joystick configuration and variant-specific teleoperation configuration.

`launch/campero_test.launch.py` is a legacy standalone test launch kept for local checks.

## 4) Relation with other packages

`campero_bringup` is the package selected by `romea_mobile_base_meta_bringup` when a Campero mobile base meta-description is used.

It delegates robot description generation to `campero_description`, live hardware communication to `campero_hardware`, bridge communication to `campero_bridge`, controller loading to `romea_mobile_base_controllers` and teleoperation to `romea_mobile_base_teleop`.
