import sys
sys.path.append('/home/rmqlife/work/ur_slam')
from ros_utils.myRobotNs import MyRobotNs
from ros_utils.myConfig import MyConfig
from pose_util import *
from pose_zoo import *
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

def execute(self, path1, path2):
    thread1 = threading.Thread(target=move_path, args=(self.arm1, path1))
    thread2 = threading.Thread(target=move_path, args=(self.arm2, path2))
    thread1.start()
    thread2.start()

def base_pose_ik(pose):
        # reset myIK's position and orientation
    transform = pose_to_SE3(pose)
    ik = MyIK_rotate(transform)
    return ik



class MyObsPlanner():
    def __init__(self, obj_path, pose_path='pose.json'):
        '''
        args:
            obj_path: str, path to the mesh file
            pose_path: str, path to the JSON file containing arm poses
        '''
        # Load poses from JSON file
        with open(pose_path, 'r') as file:
            data = json.load(file)
            arm_poses = [data['arm1_pose'], data['arm2_pose']]

        # Initialize the planner with the loaded poses
        self.obstacles = []

        # PyBullet setup
        p.connect(p.GUI)
        p.setTimeStep(1. / 240.)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load robots
        # urdf_path = "ur5e/ur5e.urdf"
        urdf_path = "ur5/urdf/ur5.urdf"

        # Robot 1
        self.robot = list()
        self.robot_ik = list()
        for i, arm_pose in enumerate(arm_poses):
            robot = p.loadURDF(urdf_path, (0, 0, 0), useFixedBase=1)
            self.robot.append(pb_ompl2.PbOMPLRobot(robot))
            p.resetBasePositionAndOrientation(robot, arm_poses[i][:3], arm_poses[i][3:])
            # reset myIK's position and orientation
            self.robot_ik.append(base_pose_ik(arm_pose))
        
        # Setup pb_ompl
        self.pb_ompl_interface = pb_ompl2.PbOMPL2(self.robot[0], self.robot[1], self.obstacles)
        self.pb_ompl_interface.set_planner("RRTstar")
        
        # Add obstacles
        self.add_mesh(obj_path, [0, 0, 0], [0, 0, 0, 1])

        self.real_robot = list()
        for ns in ['robot1', 'robot2']:
            self.real_robot.append(MyRobotNs(ns=ns))
        
    def get_joints(self):
        # output the joints of environment
        res = []
        for robot in self.robot:
            res.append(robot.get_cur_state())
        return res


    def get_poses(self):
        # output the poses by fk
        joints = self.get_joints()
        res = []
        for i, joint in enumerate(joints):
            pose = self.robot_ik[i].fk_se3(joint)
            res.append(pose)
        return res

    def set_joints(self, joint_list):
        for i, joint in enumerate(joint_list):
            if joint is not None:
                self.robot[i].set_state(joint)
        pass

    def set_poses():
        # set the env's poses by ik
        pass
    
    def sync_robot():
        # get robot's joints state
        pass

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

    def plan(self, start1, goal1, start2, goal2):
        '''
        Execute the planner and execute the planned path
        '''
        self.robot1.set_state(start1)
        self.robot2.set_state(start2)
        res, path1, path2 = self.pb_ompl_interface.plan(goal1, goal2)
        self.pb_ompl_interface.execute(path1, path2)
        return path1, path2

    


    def path_planning(self, joints1, joints2):
        path1, path2 = self.run(self.arm1.get_joints(), joints1, self.arm2.get_joints(), joints2)
        return path1, path2







if __name__ == "__main__":
    # rospy.init_node('dual_arm_bullet')
    planner = MyObsPlanner(obj_path="plydoc/mesh22.obj", pose_path='pose.json')
    # planner.add_box([0,0,0],[0.2,0.2,0.2])    
    joint_configs = MyConfig('/home/rmqlife/work/ur_slam/slam_data/joint_configs.json')
    planner.set_joints([joint_configs.get('facedown'), joint_configs.get('facedown2')])
    
    print("environment joints state", planner.get_joints())
    print("environment pose")
    for se3_pose in planner.get_poses():
        se3_pose.printline()

    # sync with real robot
    path_list = list()
    for i in [0,1]:
        init_pose = SE3_to_pose(planner.get_poses()[i])
        target_pose = init_pose.copy()
        target_pose[2] -= 0.3
        poses = circle_pose(init_pose, target_pose[:3], radius=0.1, num_points=50)
        path = planner.robot_ik[i].plan_trajectory(poses, planner.get_joints()[0])
        path_list.append(path)

    while 1:
        for j0, j1 in zip(path_list[0], path_list[1]):
            planner.set_joints([j0, j1])
            import time
            time.sleep(1./60.)


    input("break")

    # path1, path2 = planner.path_planning(joints1, planner.arm2.get_joints())

    # input("Press Enter to continue")
    # planner.execute(path1, path2)
