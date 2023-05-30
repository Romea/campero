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


# import pytest
import xml.etree.ElementTree as ET
from campero_description import urdf


def urdf_xml(mode, model):
    prefix = "robot_"
    ros_prefix = "/robot/"
    base_name = "base"
    controller_conf_yaml_file = mode + "_" + model + "_controller.yaml"
    # print(urdf(prefix, mode, model, controller_conf_yaml_file))
    return ET.fromstring(
        urdf(prefix, mode, base_name, model, controller_conf_yaml_file, ros_prefix))


def ros2_control_urdf_xml(mode, model):
    urdf_xml(mode, model)
    return ET.parse("/tmp/robot_base_ros2_control.urdf")


def test_footprint_link_name():
    assert urdf_xml("live", "rubber").find("link").get("name") == "robot_base_footprint"


def test_hardware_plugin_name():

    assert ros2_control_urdf_xml("live", "rubber").find(
        "ros2_control/hardware/plugin"
    ).text == "campero_hardware/CamperoHardware4WD"

    # assert urdf_xml("live", "mecanum").find(
    #     "ros2_control/hardware/plugin"
    # ).text == "campero_hardware/CamperoHardware4WMD"

    assert ros2_control_urdf_xml("simulation", "rubber").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface4WD"

    # assert urdf_xml("simulation", "mecanum").find(
    #     "ros2_control/hardware/plugin"
    # ).text == "romea_mobile_base_gazebo/GazeboSystemInterface4WMD"


def test_controller_filename_name():
    assert (
        urdf_xml("simulation", "rubber").find("gazebo/plugin/controller_manager_config_file").text
        == "simulation_rubber_controller.yaml"
    )
