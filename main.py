import sys
sys.path.append('/home/rmqlife/work/ur_slam/ros_utils')
from myRobotNs import MyRobotNs

import numpy as np
from ur5_dual import UR5DualPlanner
import rospy
import threading
import json



def move_path(robot, path):
    current_joint = robot.get_joints()
    print("start joint", current_joint)

    if (max(abs(path[0]-current_joint))>0.3):
        print("start joint is not correct")
    else:
        for joint_path in path:
            diff  = [abs(a-b) for a,b in zip(joint_path,current_joint)]
            joints_movement = np.max(diff)
            robot.move_joints(joint_path,duration =0.5* joints_movement, wait=False)
            current_joint = joint_path
    


class AvoidObstacles():
    def __init__(self, obj_path, pose_path='pose.json') -> None:
        '''
        args:
            json_file_path: path to the JSON file containing arm poses
        '''
        self.arm1 = MyRobotNs("robot1")
        self.arm2 = MyRobotNs("robot2")  
        

        # Load poses from JSON file
        with open(pose_path, 'r') as file:
            data = json.load(file)
            arm1_pose = data['arm1_pose']
            arm2_pose = data['arm2_pose']

        # Initialize the planner with the loaded poses
        self.env = UR5DualPlanner(arm1_pose[0:3], arm2_pose[0:3], arm1_pose[3:], arm2_pose[3:])
        self.env.add_mesh(obj_path,[0,0,0],[0,0,0,1])



    def path_planning(self, goal1, goal2, type):
        start1 = self.arm1.get_joints()
        start2 = self.arm2.get_joints()

        if type == 'point':
            goal1 = self.env.p.calculateInverseKinematics(self.env.robot1.id, 6, goal1)
            goal2 = self.env.p.calculateInverseKinematics(self.env.robot2.id, 6, goal2)

        path1, path2 = self.env.run(start1, goal1, start2, goal2)
        return path1, path2

    def execute(self, path1, path2):
        thread1 = threading.Thread(target=move_path, args=(self.arm1, path1))
        thread2 = threading.Thread(target=move_path, args=(self.arm2, path2))
        thread1.start()
        thread2.start()
    
    

if __name__ == "__main__":
    rospy.init_node('dual_arm_bullet')
    planner = AvoidObstacles(obj_path="plydoc/mesh22.obj", pose_path='./pose.json')
    # j1, j2 = planner.arm1.get_joints(),planner.arm2.get_joints()

    path1, path2 = planner.path_planning(planner.arm1.get_joints(),planner.arm2.get_joints(),'joint')

    input("press enter to continue")
    planner.execute(path1, path2)
    




    