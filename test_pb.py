import sys
import json

sys.path.append('/home/rmqlife/work/ur_slam/')

from ik_step import MyIK_rotate
from some_transform_library import pose_to_SE3  # Assuming you have a function to convert a 7D pose to SE3

# Define the joints1 variable
joints1 = [-0.11, -1.2596944014178675, 2.2710089683532715, -1.1125996748553675, 1.4878071546554565, -2.4465771357165735]

# Define the path to the JSON file containing arm poses
pose_path = '/home/rmqlife/work/ur_slam/pose.json'

# Load poses from JSON file
with open(pose_path, 'r') as file:
    data = json.load(file)
    arm1_pose = data['arm1_pose']

# Convert the 7D pose to SE3 (assuming pose_to_SE3 is a function that does this)
base_transform = pose_to_SE3(arm1_pose)

# Initialize the MyIK_rotate class with the SE3 transformation
myIK = MyIK_rotate(base_transform)

# Now you can proceed with the rest of your logic using `myIK` and `joints1`
