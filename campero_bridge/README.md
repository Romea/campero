# campero_bridge

## 1) Overview

`campero_bridge` provides the communication bridge used to operate Campero robots from the ROS2 ROMEA stack.

The bridge forwards the messages required by the live Campero hardware plugin and exposes the robot feedback expected by the ROS2 control stack.

## 2) Runtime behavior

At runtime, the bridge process connects the ROS2 Campero stack to the embedded robot interface. The live hardware plugin consumes the bridged feedback and sends commands through the bridge instead of communicating directly with the low-level controller.

## 3) Relation with other packages

`campero_bridge` is used in live mode by `campero_bringup` and `campero_hardware`. It is not required for pure simulation modes.
