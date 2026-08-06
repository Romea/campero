# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import xml.etree.ElementTree as ET

from campero_description import (
    generate_ros2_control_description,
    generate_urdf_description,
    get_configuration,
)


def urdf_xml(mode, model):
    return ET.fromstring(
        generate_urdf_description(
            "robot_",
            mode,
            "base",
            model,
            f"{mode}_{model}_controller.yaml",
            "/robot/",
        )
    )


def ros2_control_xml(mode, model):
    return ET.fromstring(
        generate_ros2_control_description("robot_", mode, "base", model)
    )


def test_configuration_contains_campero_metadata():
    configuration = get_configuration("rubber")

    assert configuration["manufacturer"] == "robotonik"
    assert configuration["model"] == "campero"
    assert configuration["version"] == "rubber"


def test_footprint_link_name():
    assert urdf_xml("live", "rubber").find("link").get("name") == "robot_base_footprint"


def test_hardware_plugin_names():
    assert (
        ros2_control_xml("live", "rubber").find("ros2_control/hardware/plugin").text
        == "campero_hardware/CamperoHardware4WD"
    )

    assert (
        ros2_control_xml("live", "mecanum").find("ros2_control/hardware/plugin").text
        == "campero_hardware/CamperoHardware4WMD"
    )

    assert (
        ros2_control_xml("simulation_gazebo_classic", "rubber")
        .find("ros2_control/hardware/plugin")
        .text
        == "romea_mobile_base_gazebo/GazeboSystemInterface4WD"
    )

    assert (
        ros2_control_xml("simulation_gazebo", "rubber")
        .find("ros2_control/hardware/plugin")
        .text
        == "romea_mobile_base_gazebo/GazeboSystemInterface"
    )


def test_simulation_plugin_uses_ros2_control_description_node():
    plugin = urdf_xml("simulation_gazebo", "rubber").find("gazebo/plugin")

    assert plugin.find("robot_param").text == "robot_description"
    assert plugin.find("robot_param_node").text == "ros2_control_description"
    assert plugin.find("parameters").text == "simulation_gazebo_rubber_controller.yaml"
