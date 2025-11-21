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

from timeit import main
import numpy as np
from isaacsim import SimulationApp

import platform
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.msg = None

    def timer_callback(self):
        if self.msg is not None:
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing image #%d' % self.i)
            self.i += 1

    def set_image(self, frame, seconds):
        # Publish camera image
        self.msg = Image()
        self.msg.header.stamp = rclpy.time.Time(seconds=seconds).to_msg()
        self.msg.header.frame_id = "camera_frame"
        self.msg.height = frame.shape[0]
        self.msg.width = frame.shape[1]
        self.msg.encoding = "rgba8"
        self.msg = CvBridge().cv2_to_imgmsg(frame, encoding="rgba8")

class TFPublisherNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('tf_publisher_node')
        self.publisher_ = self.create_publisher(TFMessage, '/tf', 10)
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.tf_msg = None

    def timer_callback(self):
        if self.tf_msg is not None:    
            # Fill in the TFMessage with dummy data
            self.publisher_.publish(self.tf_msg)
            self.get_logger().info('Publishing TF message')

    def set_tf(self, camera_pose, seconds):
        # Publish TF message
        self.tf_msg = TFMessage()

        # Fill in the TF message with camera pose information
        transform = TransformStamped()
        transform.header.stamp = rclpy.time.Time(seconds=seconds).to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "camera_frame"

        # Set translation
        transform.transform.translation.x = float(camera_pose[0][0])
        transform.transform.translation.y = float(camera_pose[0][1])
        transform.transform.translation.z = float(camera_pose[0][2])

        # Set rotation
        transform.transform.rotation.x = float(camera_pose[1][1])
        transform.transform.rotation.y = float(camera_pose[1][2])
        transform.transform.rotation.z = float(camera_pose[1][3])
        transform.transform.rotation.w = float(camera_pose[1][0])

        self.tf_msg.transforms.append(transform)

def main():
    # Exit early if running on ARM64 (aarch64) architecture
    if platform.machine().lower() in ["aarch64", "arm64"]:
        print("Livestream is not supported on ARM64 architecture. Exiting.")
        sys.exit(0)

    # This sample enables a livestream server to connect to when running headless
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

    # Start the omniverse application
    simulation_app = SimulationApp(launch_config=CONFIG)

    # Default Livestream settings
    simulation_app.set_setting("/app/window/drawMouse", True)

    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
    from isaacsim.core.api.objects.ground_plane import GroundPlane
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Sdf, UsdLux

    # --- Set up the scene ---
    # Add Ground Plane
    GroundPlane(prim_path="/World/GroundPlane", z_position=0)

    # Add Light Source
    stage = omni.usd.get_context().get_stage()
    distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    distantLight.CreateIntensityAttr(300)

    # Start a world to step simulator
    world = World(stage_units_in_meters=1.0)

    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.prims import XFormPrim
    from pxr import Gf, Usd, UsdGeom

    add_reference_to_stage(usd_path="../assets/stanford_bunny.usd", 
                        prim_path="/World/Bunny",
    )
    bunny_prim = XFormPrim(prim_path="/World/Bunny", name="Bunny_XForm")
    bunny_prim.set_world_pose(position=np.array([1.0, 0.0, 0.0]))
    bunny_prim.set_local_scale(scale=np.array([0.01, 0.01, 0.01]))

    add_reference_to_stage(usd_path="../assets/eiffel_tower.usd", 
                        prim_path="/World/Tower",
    )
    tower_prim = XFormPrim(prim_path="/World/Tower", name="Tower_XForm")
    tower_prim.set_world_pose(position=np.array([-1.0, 0.0, 0.0]))
    tower_prim.set_local_scale(scale=np.array([0.01, 0.01, 0.01]))

    add_reference_to_stage(usd_path="../assets/menger_sponge.usd",
                        prim_path="/World/Sponge",
    )
    sponge_prim = XFormPrim(prim_path="/World/Sponge", name="Sponge_XForm")
    sponge_prim.set_world_pose(position=np.array([0.0, 0.0, 0.0]))
    sponge_prim.set_local_scale(scale=np.array([0.1, 0.1, 0.1]))

    print("--- Scene Initialized ---")

    # --- Set up sensors ---
    from isaacsim.sensors.rtx import IsaacSensorCreateRtxLidar, LidarRtx

    import omni.kit.commands
    # Define the path where the Lidar will be created
    lidar_path = "/World/LidarSensor"

    # Create the RTX Lidar sensor
    # omni.kit.commands.execute(
    #     "IsaacSensorCreateRtxLidar",
    #     path=lidar_path,
    #     # parent="", # Attach to the world prim
    #     translation=Gf.Vec3d([0.0, -3.0, 1.0]),  # Position of the Lidar
    #     orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # No rotation
    #     # Additional parameters can be added for more specific configurations
    # )

    from isaacsim.sensors.camera import Camera
    from geometry_msgs.msg import TransformStamped

    camera_path = "/World/Camera"
    cam =  Camera(prim_path=camera_path, 
                name="Camera", 
                translation=np.array([0.0, -3.0, 1.0]), 
                orientation=np.array([1.0, 0.0, 0.0, 0.0]),
                frequency=30.0,
                resolution=(1280, 720),              
    )
    cam.initialize()

    world.reset()

    print("--- Sensors Initialized ---")

    # --- Initialize ROS 2 ---
    rclpy.init()

    camera_publisher_node = ImagePublisherNode()
    tf_publisher_node = TFPublisherNode()

    rclpy.spin(camera_publisher_node)
    rclpy.spin(tf_publisher_node)

    print("--- ROS 2 Publisher Initialized (Topic: /camera/image_raw, /tf) ---")

    # Enable Livestream extension
    enable_extension("omni.services.livestream.nvcf")

    # Start the simulator and run until closed
    try:
        while simulation_app._app.is_running() and not simulation_app.is_exiting():
            # print("Simulator is running... Connect to the WebRTC client now.")
            world.step(render=True)  # stepping through the simulation
            # Run in realtime mode, we don't specify the step size
            simulation_app.update()
            
            frame = cam.get_current_frame()['rgba'].astype(np.uint8) # rendering_time (float), rendering_frame (int), 'rgba' (np.ndarray)
            camera_pose = cam.get_world_pose()  # position (np.ndarray), orientation (np.ndarray), with leading quaternion w
            
            camera_publisher_node.set_image(frame, world.current_time)
            tf_publisher_node.set_tf(camera_pose, world.current_time)

    except KeyboardInterrupt:
        print("Simulation interrupted by user (ctrl+c).")

    # When ctrl+c or window closed, exit the simulation
    camera_publisher_node.destroy_node()
    tf_publisher_node.destroy_node()
    rclpy.shutdown()

    simulation_app.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()