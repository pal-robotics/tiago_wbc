# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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

import os
import yaml
import copy
import tempfile
import logging
from dataclasses import dataclass

import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetLaunchConfiguration,
    OpaqueFunction,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from controller_manager.launch_utils import generate_load_controller_launch_description
from ament_index_python.packages import get_package_share_directory

from launch_pal.arg_utils import LaunchArgumentsBase, read_launch_argument
from launch_pal.robot_arguments import CommonArgs
from tiago_pro_description.launch_arguments import TiagoArgs
from launch_pal.param_utils import parse_parametric_yaml



logger = logging.getLogger(__name__)

@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    """Defines the launch arguments used by this launch file."""

    end_effector: DeclareLaunchArgument = TiagoArgs.end_effector
    namespace: DeclareLaunchArgument          = CommonArgs.namespace
    visualizer: DeclareLaunchArgument         = DeclareLaunchArgument(
        "wbc_collision_visualizer",
        default_value="false",
        description="Enable the whole-body collision visualizer node.",
    )
    reference_type: DeclareLaunchArgument = DeclareLaunchArgument(
        "reference_type",
        default_value="InteractiveMarkerReference",
        choices=["InteractiveMarkerReference", "TopicPoseReference"],
        description="Reference type. Options: InteractiveMarkerReference, TopicPoseReference",
    )

def create_temp_file(data, suffix=".yaml") -> str:
    """Temp file for YAML or XML data."""
    temp_file = tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=suffix)

    if suffix == ".yaml":
        yaml.dump(data, temp_file, sort_keys=False)
        
    elif suffix == ".xml":
        xml_bytes = ET.tostring(data, encoding="utf-8")
        xml_str = xml_bytes.decode("utf-8") 
        temp_file.write(xml_str)
    
    temp_file.close()    
    logger.info(f"Created temporary file: {temp_file.name}")
    return temp_file.name


def create_extra_capsules_file(
    yaml_file: str, gripper_name: str, xml_path: str | None = None
) -> str:
    """
    Generate a filtered YAML file containing only the capsules for selected grippers,
    and optionally update the robot XML groups in memory.
    """
    with open(yaml_file, "r") as f:
        data = yaml.safe_load(f) or {}

    capsule_data = data.get("extra_capsule_description", {}).get("capsule_params", {})
    filtered_capsules = []

   
    if gripper_name not in capsule_data:
        logger.error(f"Gripper '{gripper_name}' not found in capsules.")
    # add exception 
    for entry in capsule_data[gripper_name]:
        entry_copy = copy.deepcopy(entry)
        filtered_capsules.append(entry_copy)

    final_yaml = {"extra_capsule_description": {"capsule_params": filtered_capsules}}

    yaml_temp_path_1 = create_temp_file(final_yaml, suffix=".yaml")
    
    xml_temp_path = None
    if xml_path:
        xml_temp_path = create_collision_operation_file(filtered_capsules, xml_path)

    return yaml_temp_path_1, xml_temp_path

def create_collision_operation_file(
   filtered_capsules, xml_path: str | None = None
) -> str:

    tree = ET.parse(xml_path)
    root = tree.getroot()

    links = [c["link_name"] for c in filtered_capsules]

    def update_group(group_name: str, links: list[str]):
        group_elem = root.find(f".//group[@name='{group_name}']")
        if group_elem is None:
            logger.warning(f"Group '{group_name}' not found in XML.")
            return
        for child in list(group_elem):
            group_elem.remove(child)
        for link_name in links:
            ET.SubElement(group_elem, "link", {"name": link_name})

    update_group("gripper_group", links)


    return create_temp_file(root, suffix=".xml")

def setup_wbc_controller(
    context, pkg_share_folder, wbc_body_yaml, extra_capsule_yaml_path, *_, **__
):
    """
    Prepare the whole-body controller configuration.
    """

    stack_configuration_path =  os.path.join(pkg_share_folder, "config", "default_kinematic_stack.yaml")

    # Read launch arguments
    end_effector   = read_launch_argument("end_effector", context)
    visualizer     = read_launch_argument("wbc_collision_visualizer", context)
    reference_type = read_launch_argument("reference_type", context)


    # Generate filtered YAML capsules
    xml_groups_path = os.path.join(pkg_share_folder, "config", "tiago_collision_operations.xml")
    extra_capsule_path, collision_op_path = create_extra_capsules_file(extra_capsule_yaml_path, end_effector, xml_groups_path)
    remappings_extra_capsule_path = {"PATH_TO_EXTRA_CAPSULE_FILE": extra_capsule_path,"PATH_TO_COLLISION_OP_FILE":collision_op_path, "REFERENCE_TYPE": reference_type}
    stack_yaml = parse_parametric_yaml(source_files=[stack_configuration_path], param_rewrites=remappings_extra_capsule_path)
    remappings_stack_path = {"PATH_TO_STACK_FILE": stack_yaml}
    wbc_new_yaml = parse_parametric_yaml(source_files=[wbc_body_yaml], param_rewrites=remappings_stack_path)

    # Setup WBC controller group
    wbc_controller = setup_wbc_files(context, wbc_new_yaml)

    return [
        SetLaunchConfiguration("extra_capsule_file", extra_capsule_path),
        Node(
            package="capsule_collision",
            executable="capsule_self_collision",
            name="capsule_self_collision",
            parameters=[
                {
                    "collision_decomposition_description": os.path.join(
                        pkg_share_folder, "config", "tiago_pro_capsule_params.yaml"
                    )
                },
                {"extra_capsule_descriptions": LaunchConfiguration("extra_capsule_file")},
            ],
            condition=IfCondition(visualizer),
        ),
        wbc_controller,
    ]


def setup_wbc_files(context, wbc_yaml):
    """Load the `pal_wbc_controller` using the given parameter file."""
    return GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name="pal_wbc_controller",
                controller_params_file=wbc_yaml,
                extra_spawner_args=["--inactive"],
            ),
        ],
        forwarding=False,
    )


def declare_actions(launch_description: LaunchDescription):
    """Declare main actions for the launch description."""
    pkg_share_folder = get_package_share_directory("tiago_wbc")
    whole_body_yaml = os.path.join(pkg_share_folder, "config", "whole_body_kinematic_controller.yaml")
    extra_capsule_yaml_path = os.path.join(pkg_share_folder, "config", "tiago_pro_extra_collision_descriptions.yaml")

    launch_description.add_action(
        OpaqueFunction(
            function=setup_wbc_controller,
            kwargs={
                "pkg_share_folder": pkg_share_folder,
                "wbc_body_yaml": whole_body_yaml,
                "extra_capsule_yaml_path": extra_capsule_yaml_path,
            },
        )
    )


def generate_launch_description() -> LaunchDescription:
    """Entry point for ROS 2 launch system."""
    ld = LaunchDescription()
    LaunchArguments().add_to_launch_description(ld)
    declare_actions(ld)
    return ld
