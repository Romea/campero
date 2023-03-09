# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license


# import pytest
import xml.etree.ElementTree as ET
from campero_description import urdf


def urdf_xml(mode, model):
    prefix = "robot_"
    ros_namespace = "\robot"
    controller_conf_yaml_file = mode + "_" + model + "_controller.yaml"
    # print(urdf(prefix, mode, model, controller_conf_yaml_file))
    return ET.fromstring(urdf(prefix, mode, model, controller_conf_yaml_file, ros_namespace))


def test_footprint_link_name():
    assert urdf_xml("live", "rubber").find("link").get("name") == "robot_base_footprint"


def test_hardware_plugin_name():

    assert urdf_xml("live", "rubber").find(
        "ros2_control/hardware/plugin"
    ).text == "campero_hardware/CamperoHardware4WD"

    # assert urdf_xml("live", "mecanum").find(
    #     "ros2_control/hardware/plugin"
    # ).text == "campero_hardware/CamperoHardware4WMD"

    assert urdf_xml("simulation", "rubber").find(
        "ros2_control/hardware/plugin"
    ).text == "romea_mobile_base_gazebo/GazeboSystemInterface4WD"

    # assert urdf_xml("simulation", "mecanum").find(
    #     "ros2_control/hardware/plugin"
    # ).text == "romea_mobile_base_gazebo/GazeboSystemInterface4WMD"


def test_controller_filename_name():
    assert (
        urdf_xml("simulation", "rubber").find("gazebo/plugin/parameters").text
        == "simulation_rubber_controller.yaml"
    )
