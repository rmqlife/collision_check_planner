#!/usr/bin/env python3

import json
import numpy as np
from scipy.spatial.transform import Rotation

def pose2matrix(pose):
    """Convert a pose (position + quaternion) to a 4x4 transformation matrix."""
    position = pose[:3]
    orientation = pose[3:]
    rotation_matrix = Rotation.from_quat(orientation).as_matrix()
    
    matrix = np.eye(4)
    matrix[:3, :3] = rotation_matrix
    matrix[:3, 3] = position
    
    return matrix

def matrix2pose(matrix):
    """Convert a 4x4 transformation matrix to a pose (position + quaternion)."""
    position = matrix[:3, 3]
    rotation_matrix = matrix[:3, :3]
    orientation = Rotation.from_matrix(rotation_matrix).as_quat()
    
    return np.concatenate([position, orientation]).tolist()

def save_pose_to_json(arm1_pose, arm2_pose, filename='pose.json'):
    """Save arm poses to a JSON file."""
    data = {
        'arm1_pose': arm1_pose,
        'arm2_pose': arm2_pose
    }
    print(data)
    with open(filename, 'w') as f:
        json.dump(data, f, indent=4)

if __name__ == "__main__":
    arm_1_position = [-0.6226, -0.03173, 0.1716]
    arm_1_orientation = [0.92061594, 0.00544071, -0.00534512, 0.39039482]
    arm1_pose = arm_1_position + arm_1_orientation

    # Example transformation matrix (replace with actual computation if necessary)
    arm_transformation = np.array([
        [0.996, 0.0683, -0.05744, 0.02818],
        [-0.057, -0.00844, -0.9983, -0.3216],
        [-0.06867, 0.9976, -0.004513, -0.3276],
        [0, 0, 0, 1]
    ])
    
    arm2_matrix = pose2matrix(arm1_pose) @ arm_transformation
    arm2_pose = matrix2pose(arm2_matrix)

    print(arm2_pose, arm1_pose)
    save_pose_to_json(arm1_pose, arm2_pose)
