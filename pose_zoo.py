import numpy as np
import math

unit_vector = [1,0,0]

def rectangle_points(center, x, y):
    points = []
    for i, j in [(-x,-y), (-x, y), (x,y), (x, -y),(-x,-y)]:
        point = center.copy()
        point[0]+=i
        point[1]+=j
        points.append(point)
    
    # connect start to end    
    return points



def vec_to_quat(v1, v2):
    import tf.transformations as transformations
    # Ensure input vectors are unit vectors
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    # Compute the quaternion representing the rotation from v1 to v2
    q = transformations.quaternion_about_axis(np.arccos(np.dot(v1, v2)), np.cross(v1, v2))
    # q = swap_quat(q)
    return q


def circle_points(center, radius=0.1, num_points=20):
    points = []

    for i in range(num_points):
        theta = 2.0 * math.pi * i / num_points
        delta_x = radius * math.cos(theta)
        delta_y = radius * math.sin(theta)

        point = np.array([center[0] + delta_x, center[1] + delta_y, center[2]])
        points.append(point)

    return points


def circle_pose(center, toward,  radius, num_points):
    points = circle_points(center, radius=radius, num_points=num_points)
    for i in range(len(points)):
        quat = vec_to_quat(unit_vector,  toward-points[i])
        points[i] =  list(points[i][:3]) + list(quat)

    return np.array(points)


if __name__=="__main__":
    init_pose = (0, 0, 0)
    target_pose = (0, 0, -0.5)
    points = circle_pose(init_pose, target_pose, radius=0.3, num_points=50)
    from pose_util import *
    visualize_poses(points, label="points to plan", color='y', autoscale=False, ax=None)
    plt.show()  