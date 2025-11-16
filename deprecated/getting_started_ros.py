# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
from isaacsim import SimulationApp

# CONFIG = {
#     "livestream": True,
#     "width": 1280,
#     "height": 720,
#     "window_width": 1920,
#     "window_height": 1080,
#     "headless": False,       # 렌더링/스트리밍 켜기
#     "hide_ui": True,         # [필수] 로컬 GUI 창 시도 끄기
#     "renderer": "RaytracedLighting",
#     "display_options": 3286,
# }

# simulation_app = SimulationApp({"headless": True})

CONFIG = {
    "width": 1280,
    "height": 720,
    "window_width": 1920,
    "window_height": 1080,
    "headless": True,
    "hide_ui": False,  # Show the GUI
    "renderer": "RaytracedLighting",
    "display_options": 3286,  # Set display options to show default grid
}

simulation_app = SimulationApp(CONFIG)

from isaacsim.core.utils.extensions import enable_extension

# Default Livestream settings
simulation_app.set_setting("/app/window/drawMouse", True)

# Enable Livestream extension
enable_extension("omni.services.livestream.nvcf")

# 3. [가장 중요] 이제 ROS 2와 Isaac Sim 모듈을 import 합니다.
import rclpy
from std_msgs.msg import String
import time

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.objects.ground_plane import GroundPlane
from pxr import Sdf, UsdLux

# --- ROS 2 초기화 ---
rclpy.init()
node = rclpy.create_node('isaac_sim_publisher_node')
publisher = node.create_publisher(String, 'isaac_topic', 10)
msg = String()
print("--- ROS 2 Publisher Initialized (Topic: /isaac_topic) ---")
# ---------------------


# --- 씬 생성 (기존 코드) ---
GroundPlane(prim_path="/World/GroundPlane", z_position=0)
stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(300)
DynamicCuboid(
    prim_path="/dynamic_cube",
    name="dynamic_cube",
    position=np.array([0, 0, 1.5]),
    size=0.3,
    color=np.array([0, 255, 255]),
)
my_world = World(stage_units_in_meters=1.0)
my_world.reset()
# -------------------------

print("Simulator is running... Connect to the WebRTC client now.")

count = 0
while simulation_app._app.is_running() and not simulation_app.is_exiting():
    # 4. 시뮬레이션 스텝 실행
    my_world.step(render=True)
    
    # 5. [필수] ROS 2 노드도 스핀을 돌려줍니다.
    rclpy.spin_once(node, timeout_sec=0)

    # 6. Publish ROS 2 message every second
    if count % 60 == 0:  # Publish message every 60 steps (approximately 1 second if step rate is 60 Hz)
        msg.data = f"Hello from Isaac Sim! Step: {count}"
        publisher.publish(msg)
        print(f"Published: {msg.data}")
    count += 1
    
    simulation_app.update()

simulation_app.close()