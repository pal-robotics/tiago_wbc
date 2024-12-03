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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from controller_manager.launch_utils import generate_load_controller_launch_description

def declare_actions(launch_description: LaunchDescription):

    pkg_share_folder = get_package_share_directory("tiago_wbc")

    pal_wbc_controller = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name="pal_wbc_controller",
                controller_params_file=os.path.join(
                    pkg_share_folder, "config", "whole_body_dynamic_controller.yaml"
                ),
                extra_spawner_args=["--unload-on-kill"],
            )
        ],
    )

    launch_description.add_action(pal_wbc_controller)

    return

def generate_launch_description():

    ld = LaunchDescription()

    declare_actions(ld)

    return ld
