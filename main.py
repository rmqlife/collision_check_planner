import sys
sys.path.append('/home/rmqlife/work/catkin_ur5/src/teleop/src')
sys.path.append('/home/rmqlife/work/collision_check/ompl/py-bindings')

import numpy as np
from ur5_dual import UR5DualPlanner
from move_ur5 import myur5
from scipy.spatial.transform import Rotation
from spatialmath import SE3
import threading

def pose2matrix(pose):
    position  = pose[0:3]
    orientation = pose[3:]
    r = Rotation.from_quat(orientation)
    matrix = r.as_matrix()
    out = np.zeros((4,4))
    out[0:3,0:3] = matrix
    out[0:3,3] = position
    return out

def matrix2pose(matrix):
    position = matrix[0:3, 3]
    r = Rotation.from_matrix(matrix[0:3,0:3])
    orientation = r.as_quat()
    pose = np.concatenate((position, orientation))
    return pose

class AvoidObstacles():
    def __init__(self,arm1_poses) -> None:
        '''
        args:
            env: UR5DualEnv class
            arm1_poses: list of floats, [x,y,z,orientation]
            arm2_poses: list of floats, [x,y,z,orientation]
        '''

        arm_transformation = [   [0.996     ,0.0683  , -0.05744  , 0.02818],   
                                [-0.057   , -0.00844,  -0.9983  , -0.3216 ], 
                                [-0.06867  , 0.9976  , -0.004513, -0.3276], 
                                [0        , 0       ,  0       ,  1   ]]
        
        arm_transformation= np.array(arm_transformation)# constant transformation between two arms
        arm2_matrix =   np.array(pose2matrix(arm1_poses)) @ arm_transformation
        arm2_poses = matrix2pose(arm2_matrix)
        print(arm1_poses[0:3])
        print(arm2_poses[0:3])
        self.env = UR5DualPlanner(arm1_poses[0:3],arm2_poses[0:3],arm1_poses[3:],arm2_poses[3:])
        
        self.robot1 = self.env.robot1
        self.id1 = self.robot1.id
        self.robot2 = self.env.robot2
        self.id2 = self.robot2.id
        self.match_state()
    
        
    def add_obstacles(self,obstacles_path):
        '''
        add obstacles to the environment
        
        '''
        self.env.add_mesh(obstacles_path,[0,0,0],[0,0,0,1])


    def path_planning(self,goal1,goal2,type = 'joint'):
        '''
        path planning for two robots
        
        args:
            start1: list of floats, start configuration of robot1
            start2: list of floats, start configuration of robot2
            goal1: list of floats, goal configuration of robot1
            goal2: list of floats, goal configuration of robot2
        
        return:
            path1: list of list of floats, path found by the planner for robot1
            path2: list of list of floats, path found by the planner for robot2
        '''
        
        self.robot1.set_state(self.start1)
        self.robot2.set_state(self.start2)

        if type == 'joint':
            goal1_joint = goal1
            goal2_joint = goal2
        elif type == 'point':
            goal1_joint = self.env.p.calculateInverseKinematics(self.id1, 6,goal1)
            goal2_joint = self.env.p.calculateInverseKinematics(self.id2, 6,goal2)
        else:
            raise ValueError("type should be 'joint' or 'point'")
        
        self.path1,self.path2 = self.env.run(self.start1,goal1_joint,self.start2,goal2_joint)


    def match_state(self):
        '''
        match the real state of the two robots
        and get the current state of the two robots
        '''
        self.ur5_move1 = myur5("robot1")
        self.ur5_move2 = myur5("robot2")            
        self.start1 = self.ur5_move1.get_current_joint()
        self.start2 = self.ur5_move2.get_current_joint()

    def execute(self):
        thread1 = threading.Thread(target=self.ur5_move1.move_, args=(self.path1,))
        thread2 = threading.Thread(target=self.ur5_move2.move_, args=(self.path2,))
        thread1.start()
        thread2.start()
    


if __name__ == "__main__":

    arm_1_position =  [-0.6226 , -0.03173 , 0.1716 ]
    arm_1_orientation = [ 0.92061594 , 0.00544071 ,-0.00534512,  0.39039482]
    arm1_poses = arm_1_position+arm_1_orientation

    avoid_obstacles = AvoidObstacles(arm1_poses)
    avoid_obstacles.add_obstacles("plydoc/mesh22.obj")
    # avoid_obstacles.path_planning([0.1,0.1,0.2],[-0.1,-0.1,-0.3],'point')

    input("press enter to continue")
    avoid_obstacles.execute()
    




    