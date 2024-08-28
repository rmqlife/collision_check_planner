import rospy
from octomap_msgs.msg import Octomap
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

global octomap_data
octomap_data = None

def callback(data):
    global octomap_data
    rospy.loginfo("Received Octomap data")
    octomap_data = data.data

def listener():
    rospy.init_node('octomap_listener', anonymous=True)
    rospy.Subscriber('/rtabmap/octomap_full', Octomap, callback)

if __name__ == '__main__':
    listener()

