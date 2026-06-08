# campero

## Overview

`campero` groups the ROS2 packages that describe, launch and control Campero mobile bases in live and simulation modes.

This repository-level README gives a map of the stack. Detailed information about each package can be found in the corresponding package README.

## Packages

| Package | Role |
| --- | --- |
| `campero` | Metapackage that groups the Campero ROS2 packages. |
| `campero_description` | Robot-specific description layer for Campero variants, including configuration files, URDF/Xacro descriptions, meshes and ros2_control descriptions. |
| `campero_bringup` | Main integration entry point for generating Campero configuration files, URDF descriptions, ros2_control descriptions and launch files. |
| `campero_hardware` | Live `ros2_control` hardware plugins for Campero mobile bases. |
| `campero_bridge` | ROS1 / ROS2 bridge used by live Campero robots to exchange commands and feedback with the low-level controller. |

## Usage

In most cases, start with `campero_bringup`. It is the user-facing entry point of the stack and the package used by `romea_mobile_base_meta_bringup` when a Campero model is selected from a mobile base meta-description.

The Campero stack is a robot-specific specialization of `romea_mobile_base`. The supported variants are `rubber` and `mecanum`; `rubber` uses the `4WD` architecture and skid-steering commands, while `mecanum` uses the `4WMD` architecture and omni-steering commands. `campero_description` provides the concrete geometry and generated descriptions, `campero_hardware` provides the live hardware implementation, `campero_bridge` connects the ROS2 stack to the embedded robot interface, and `campero_bringup` connects these pieces to the generic mobile base launch workflow.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `campero` project was developed by Jean Laneurit in the context of ROMEA projects involving the TSCF research unit.

## Contact

For questions or comments about this project, please contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
