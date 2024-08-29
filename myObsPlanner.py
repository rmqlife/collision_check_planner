import sys
sys.path.append('/home/rmqlife/work/ur_slam')
from ros_utils.myRobotNs import MyRobotNs
from pose_util import *
from ik_step import MyIK_rotate

sys.path.append('/home/rmqlife/work/collision_check/ompl/py-bindings')
import pybullet as p
import pybullet_data
import pb_ompl2
import numpy as np
import rospy
import threading
import json
from spatialmath import SE3

def move_path(robot, path):
    current_joint = robot.get_joints()
    print("start joint", current_joint)

    if max(abs(path[0] - current_joint)) > 0.3:
        print("start joint is not correct")
    else:
        for joint_path in path:
            diff = [abs(a - b) for a, b in zip(joint_path, current_joint)]
            joints_movement = np.max(diff)
            robot.move_joints(joint_path, duration=3 * joints_movement, wait=False)
            current_joint = joint_path

def robotik(robotid,point,q0):
    '''
    args:
        robotid: int, robot id
        point: list of floats, [x,y,z]
    return:
        joint: list of floats, joint configuration
    '''
    base_position, base_orientation = p.getBasePositionAndOrientation(robotid)
    pose = base_position + base_orientation  
    robot_transfomration = pose_to_SE3(pose)
    myik = MyIK_rotate(robot_transfomration)
    point = SE3(point)
    joint = myik.ik_point(point,q0)
    print(joint)
    return joint

class DualArmPlanner():
    def __init__(self, obj_path, pose_path='pose.json'):
        '''
        args:
            obj_path: str, path to the mesh file
            pose_path: str, path to the JSON file containing arm poses
        '''
        self.arm1 = MyRobotNs("robot1")
        self.arm2 = MyRobotNs("robot2")

        # Load poses from JSON file
        with open(pose_path, 'r') as file:
            data = json.load(file)
            arm1_pose = data['arm1_pose']
            arm2_pose = data['arm2_pose']

        # Initialize the planner with the loaded poses
        self.obstacles = []

        # PyBullet setup
        p.connect(p.GUI)
        p.setTimeStep(1. / 240.)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load robots
        urdf_path = "ur5e/ur5e.urdf"

        # Robot 1
        robot_1 = p.loadURDF(urdf_path, (0, 0, 0), useFixedBase=1)
        self.robot1 = pb_ompl2.PbOMPLRobot(robot_1)

        # Robot 2
        robot_2 = p.loadURDF(urdf_path, (0, 0, 0), useFixedBase=1)
        self.robot2 = pb_ompl2.PbOMPLRobot(robot_2)

        # Reset robot position and orientation
        p.resetBasePositionAndOrientation(robot_1, arm1_pose[:3], arm1_pose[3:])
        p.resetBasePositionAndOrientation(robot_2, arm2_pose[:3], arm2_pose[3:])

        # Setup pb_ompl
        self.pb_ompl_interface = pb_ompl2.PbOMPL2(self.robot1, self.robot2, self.obstacles)
        self.pb_ompl_interface.set_planner("RRTstar")

        # Add obstacles
        self.add_mesh(obj_path, [0, 0, 0], [0, 0, 0, 1])

    def clear_obstacles(self):
        for obstacle in self.obstacles:
            p.removeBody(obstacle)

    def update_obstacles(self):
        '''
        Update the obstacles in the environment
        '''
        self.pb_ompl_interface.set_obstacles(self.obstacles)

    def add_mesh(self, file_path, posi, ori):
        '''
        Add a mesh (.obj) to the environment
        args:
            file_path: str, path to the mesh file
            posi: list of 3 floats, position of the mesh
            ori: list of 4 floats, orientation of the mesh (quaternion xyzw)
        '''
        meshId = p.createCollisionShape(p.GEOM_MESH, fileName=file_path, meshScale=[1, 1, 1], flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        mesh_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=meshId, basePosition=posi, baseOrientation=ori)
        self.obstacles.append(mesh_body)
        self.update_obstacles()
        return mesh_body

    def add_box(self, box_pos, half_box_size):
        '''
        Add box as obstacle
        args:
            box_pos: list, [x,y,z]
            half_box_size: list, [x,y,z]
        '''
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)
        self.obstacles.append(box_id)
        self.update_obstacles()
        return box_id

    def run(self, start1, goal1, start2, goal2):
        '''
        Execute the planner and execute the planned path
        '''
        self.robot1.set_state(start1)
        self.robot2.set_state(start2)
        res, path1, path2 = self.pb_ompl_interface.plan(goal1, goal2)
        print(res, path1, path2)
        self.pb_ompl_interface.execute(path1, path2)
        return path1, path2

    def path_planning(self, joints1, joints2):
        path1, path2 = self.run(self.arm1.get_joints(), joints1, self.arm2.get_joints(), joints2)
        return path1, path2

    def execute(self, path1, path2):
        thread1 = threading.Thread(target=move_path, args=(self.arm1, path1))
        thread2 = threading.Thread(target=move_path, args=(self.arm2, path2))
        thread1.start()
        thread2.start()

if __name__ == "__main__":
    # rospy.init_node('dual_arm_bullet')
    planner = DualArmPlanner(obj_path="plydoc/mesh22.obj", pose_path='pose.json')
    # planner.add_box([0,0,0],[0.2,0.2,0.2])

    joint = robotik(planner.robot1.id,[-0.1,-0.1,0],[0,0,0,0,0,0])
    planner.robot1.set_state(joint)

    input("break")
    



    # path1, path2 = planner.path_planning(joints1, planner.arm2.get_joints())

    # input("Press Enter to continue")
    # planner.execute(path1, path2)
