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
    
class UR5DualPlanner():
    def __init__(self, arm1_pose, arm2_pose):
        '''
        args:
            arm1_pose, arm2_pose: list, [x,y,z],[x,y,z,w]
        '''

        self.obstacles = [] 

        p.connect(p.GUI)
        # p.connect(p.DIRECT)
        # p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # load robot
        urdf_path = "ur5e/ur5e.urdf"
  

        #robot1
        robot_1 = p.loadURDF(urdf_path, (0,0,0), useFixedBase = 1)
        robot1 = pb_ompl2.PbOMPLRobot(robot_1)
        self.robot1 = robot1

        #robot2
        robot_2 = p.loadURDF(urdf_path, (0,0,0), useFixedBase = 1)
        robot2 = pb_ompl2.PbOMPLRobot(robot_2)
        self.robot2 = robot2

        # reset robot position and orientation
        p.resetBasePositionAndOrientation(robot_1, arm1_pose[:3], arm1_pose[3:])
        p.resetBasePositionAndOrientation(robot_2, arm2_pose[:3], arm2_pose[3:])


        # setup pb_ompl
        self.pb_ompl_interface = pb_ompl2.PbOMPL2(self.robot1,self.robot2, self.obstacles)


        ## set planner/ if not set, it will use the default planner
        # possible values: "RRT", "RRTConnect", "RRTstar", "FMT", "PRM" "EST", "BITstar","BFMT"
        self.pb_ompl_interface.set_planner("RRTstar")


        # add obstacles
        self.update_obstacles()
        self.p = p

    def clear_obstacles(self):
        
        for obstacle in self.obstacles:
            p.removeBody(obstacle)


    def update_obstacles(self):
        '''
        update the obstacles in the environment

        '''
        self.pb_ompl_interface.set_obstacles(self.obstacles)

    def add_mesh(self,file_path,posi,ori):
        '''
        Add a mesh(.obj) to the environment
        args:
            file_path: str, path to the mesh file
            posi: list of 3 floats, position of the mesh
            ori: list of 4 floats, orientation of the mesh(quaternion xyzw)
        '''

        meshId = p.createCollisionShape(p.GEOM_MESH,fileName=file_path, meshScale=[1,1,1],flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        # create a multi body with the collision shape
        mesh_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=meshId, basePosition=posi, baseOrientation= ori )    
        self.obstacles.append(mesh_body)
        self.pb_ompl_interface.set_obstacles(self.obstacles)
        return mesh_body


    
    def add_box(self, box_pos, half_box_size):
        '''
        add box as obstacle
        args:
            box_pos: list, [x,y,z]
            half_box_size: list, [x,y,z]
    
        '''
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        self.pb_ompl_interface.set_obstacles(self.obstacles)
        return box_id     
    


    def run(self, start1, goal1, start2, goal2):
        '''
        execute the planner and execute the planned path
        '''
        self.robot1.set_state(start1)
        self.robot2.set_state(start2)
        res, path1, path2 = self.pb_ompl_interface.plan(goal1,goal2)
        print(res, path1, path2)
        # execute the planned path
        # while 1:
        self.pb_ompl_interface.execute(path1, path2)
        return path1, path2
        

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
        self.env = UR5DualPlanner(arm1_pose, arm2_pose)
        self.env.add_mesh(obj_path,[0,0,0],[0,0,0,1])



    def path_planning(self, joints1, joints2):
        path1, path2 = self.env.run(self.arm1.get_joints(), 
                                    joints1, 
                                    self.arm2.get_joints(),
                                    joints2)
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
    




    