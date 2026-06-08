# campero_description

## 1) Overview

`campero_description` provides the robot-specific description layer for Campero mobile bases.

It extends `romea_mobile_base_description` with the concrete configuration, URDF/Xacro files, meshes and `ros2_control` descriptions required to instantiate Campero robots.

The supported Campero variants are:

| Variant | Configuration file | Mobile base architecture | Command type |
|---|---|---|---|
| `rubber` | `config/campero_rubber.yaml` | `4WD` | `skid_steering` |
| `mecanum` | `config/campero_mecanum.yaml` | `4WMD` | `omni_steering` |

## 2) Robot configuration

The `config/` directory contains the robot configuration files used by the Python API and by the Xacro descriptions.

Each Campero configuration follows the structure defined by `romea_mobile_base_description` and contains:

* the mobile base architecture;
* geometry, wheel dimensions and chassis bounding box;
* wheel speed command limits;
* sensor feedback characteristics;
* inertia and control point;
* link and joint names used in the URDF and `ros2_control` descriptions.

The package also provides one teleoperation configuration per variant: `config/teleop_rubber.yaml` and `config/teleop_mecanum.yaml`.

## 3) URDF and ros2_control descriptions

The URDF description is built from:

| Path | Role |
|---|---|
| `urdf/campero_rubber.urdf.xacro` | entry point for the `rubber` variant |
| `urdf/campero_mecanum.urdf.xacro` | entry point for the `mecanum` variant |
| `urdf/campero.xacro` | common Campero mobile base macro |
| `urdf/campero.simulation.xacro` | simulator-specific Gazebo or Gazebo Classic plugin insertion |
| `meshes/` | chassis and wheel visual meshes |

The `ros2_control` description is built from:

| Path | Role |
|---|---|
| `ros2_control/campero_rubber.ros2_control.urdf.xacro` | entry point for the `rubber` variant |
| `ros2_control/campero_mecanum.ros2_control.urdf.xacro` | entry point for the `mecanum` variant |
| `ros2_control/campero.ros2_control.xacro` | common Campero `ros2_control` macro |

Depending on the selected mode, the `ros2_control` description selects the live Campero hardware plugin or a simulation plugin from the mobile base stack.

## 4) Python API

The installed Python module provides helper functions used by `campero_bringup` and by the meta-bringup workflow.

| Function | Purpose |
|---|---|
| `get_specifications_path_file(robot_model)` | returns the configuration file path for `rubber` or `mecanum` |
| `get_specifications_configuration(robot_model)` | loads the full robot configuration |
| `get_configuration(robot_model)` | returns the compact mobile base configuration completed with manufacturer, model and version |
| `generate_configuration_file(configuration, extended)` | serializes the compact configuration |
| `generate_urdf_description(...)` | generates the Campero URDF description |
| `generate_ros2_control_description(...)` | generates the Campero `ros2_control` description |

## 5) Relation with other packages

`campero_description` is the Campero specialization of `romea_mobile_base_description`:

* `romea_mobile_base_description` provides the generic mobile base configuration and chassis templates;
* `campero_description` provides the Campero configuration, meshes and Xacro specialization;
* `campero_bringup` uses the Python API to generate the artifacts required by the generic mobile base meta-bringup workflow.
