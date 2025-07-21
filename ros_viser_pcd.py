"""Point cloud visualization

Visualize 3D point clouds with colors.

Point clouds are fundamental for many 3D computer vision applications like SLAM, 3D reconstruction, and neural radiance fields. This example demonstrates how to use :meth:`viser.SceneApi.add_point_cloud` to display point clouds with per-point colors.

The example shows two different point clouds:

1. A **spiral point cloud** with height-based color gradient (blue to red)
2. A **random noise cloud** with random colors for each point

We also add a coordinate frame using :meth:`viser.SceneApi.add_frame` to provide spatial reference. Point clouds support various parameters like ``point_size`` to control visual appearance.
"""
######################### IT WORKS!!! #########################


import numpy as np
import open3d as o3d
from sklearn import preprocessing
import glob
import os
import time
import copy
import rclpy
from rclpy.node import Node
from concurrent.futures import ThreadPoolExecutor

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

import viser
from viser import _messages
from viser import _scene_handles

class PCDSubscriber(Node):

    def __init__(self):
        super().__init__('pcd_subscriber')
        self.server = viser.ViserServer()
        self.server.gui.configure_theme(brand_color=(255, 0, 0))

        self.exec = ThreadPoolExecutor(max_workers=1)

        @self.server.on_client_connect
        def _(client: viser.ClientHandle) -> None:
            # Draw a rectangle
            click_button_handle = client.gui.add_button(
                "Draw rectangle", icon=viser.Icon.POINTER
            )

            @click_button_handle.on_click
            def _(_):
                click_button_handle.disabled = True

                @client.scene.on_pointer_event(event_type="rect-select")
                def _(event: viser.ScenePointerEvent) -> None:
                    (x_min, y_min), (x_max, y_max) = event.screen_pos
                    print(f"Region selected: {event.screen_pos}")

                @client.scene.on_pointer_callback_removed
                def _():
                    click_button_handle.disabled = False

        self.cloud_counter = 0
        self.full_pcd = o3d.geometry.PointCloud()
        self.full_down_pcd = o3d.geometry.PointCloud()

        self.subscription = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.pcd_listener_callback,
            10)
        self.subscription

    def update_viser(self):
        # Downsample
        down_pcd = self.full_pcd.voxel_down_sample(voxel_size=0.5)
        num_points_down = np.size(np.asarray(down_pcd.points), axis=0)
        self.full_down_pcd.points.extend(np.asarray(down_pcd.points))
        print("Downsampled cloud has " + str(num_points_down) + " points. Making it available to client...")

        # Color gradient from blue (bottom) to red (top).
        normalize_z = preprocessing.minmax_scale(np.asarray(down_pcd.points)[:,2], (0, 255))

        colors = np.zeros((num_points_down, 3), dtype=np.uint8)
        colors[:, 0] = (normalize_z).astype(np.uint8)  # Red channel.
        colors[:, 2] = (255 - normalize_z).astype(np.uint8)  # Blue channel.

        # Add the point cloud to the scene.
        self.server.scene.add_point_cloud(
            name= "Real-time cloud", # "cloud #" + str(self.cloud_counter),
            points=np.asarray(down_pcd.points),
            colors=colors,
            point_size=0.05,
        )
        

    def pcd_listener_callback(self, msg: PointCloud2):
        cloud_in = pc2.read_points_numpy(msg)
        self.full_pcd.points.extend(cloud_in[:, :3])
        self.cloud_counter = self.cloud_counter + 1
        print("Arrived cloud #" + str(self.cloud_counter))

        num_points = np.size(np.asarray(self.full_pcd.points), axis=0)
        print("Buffer cloud now has " + str(num_points) + " points")
        if np.mod(self.cloud_counter, 10) == 0:
            a = self.exec.submit(self.update_viser)
            # self.update_viser()

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = PCDSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()