# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

import xacro

from ament_index_python.packages import get_package_share_directory


def urdf(prefix, mode, robot_model, controller_conf_yaml_file, ros_namespace):

    xacro_file = (
        get_package_share_directory("campero_description")
        + "/urdf/campero_"
        + robot_model
        + ".urdf.xacro"
    )

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "controller_conf_yaml_file": controller_conf_yaml_file,
            "ros_namespace": ros_namespace
        },
    )

    return urdf_xml.toprettyxml()
