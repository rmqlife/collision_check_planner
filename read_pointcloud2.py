#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from  pointcloud_util import *
import time

class PointCloudListener:
    def __init__(self):
        self.pointcloud_data = None
        self.subscriber = None

    def callback(self, data):
        rospy.loginfo("Received PointCloud2 data")
        # Convert the PointCloud2 data to a list of points
        points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
        self.pointcloud_data = np.array(points)

    def visualize_pcd(self):
        # Convert numpy array to Open3D PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.pointcloud_data)
        
        # Visualize the point cloud
        o3d.visualization.draw_geometries([pcd])

    def start_listening(self):
        # Initialize the ROS node
        rospy.init_node('cloud_obstacles_listener', anonymous=True)
        # Subscribe to the /rtabmap/cloud_obstacles topic
        self.subscriber = rospy.Subscriber('/rtabmap/cloud_obstacles', PointCloud2, self.callback)
        
        # Keep the node running
        rospy.spin()

    def get_pcd(self):
        # Return the latest point cloud data
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.pointcloud_data)
        return pcd

    def run(self):
        # Start listening for point cloud data
        self.start_listening()

        # Visualize the point cloud data after rospy.spin() exits
        if self.pointcloud_data is not None:
            rospy.loginfo("Visualizing point cloud data...")
            self.visualize_pcd()
        else:
            rospy.logwarn("No point cloud data available for visualization.")



if __name__ == "__main__":
    # listener()
    # time.sleep(0.5)
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(pointcloud_data)
    # pc1 = pointcloud(pcd)
    # mesh = pc1.create_mesh()
    # pc1.visualize(mesh)
    pcl = PointCloudListener()
    pcl.run()




