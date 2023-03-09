# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license


from ament_index_python.packages import get_package_share_directory
import campero_description


def urdf_description(prefix, mode, model, ros_namespace):

    controller_manager_yaml_file = (
        get_package_share_directory("campero_bringup")
        + "/config/controller_manager.yaml"
    )

    return campero_description.urdf(
        prefix,
        mode,
        model,
        controller_manager_yaml_file,
        ros_namespace
    )
