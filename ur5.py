'''
export PYTHONPATH=$PYTHONPATH:/home/rmqlife/work/collision_check/ompl/py-bindings 
'''

import pybullet as p
import pybullet_data
import numpy as np
import pb_ompl
from move_ur5 import *


class UR5Demo():
    def __init__(self,posi,ori) -> None:
        self.obstacles = []
        
        p.connect(p.GUI)
        # p.connect(p.DIRECT)
        # p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())


        # load robot
        urdf_path = "ur5e/ur5e.urdf"


        robot_id = p.loadURDF(urdf_path,basePosition=posi, baseOrientation= ori, useFixedBase = 1)
        self.robotid = robot_id
        robot = pb_ompl.PbOMPLRobot(robot_id)
        self.robot = robot


        # setup pb_ompl
        self.pb_ompl_interface = pb_ompl.PbOMPL(self.robot, self.obstacles)


        # set planner
        # possible values: "RRT", "RRTConnect", "RRTstar", "FMT", "PRM" "EST", "BITstar","BFMT"
        self.pb_ompl_interface.set_planner("BITstar")


        # add obstacles
        self.update_obstacles()
        self.path = []



    def clear_obstacles(self):
        for obstacle in self.obstacles:
            p.removeBody(obstacle)


    def update_obstacles(self):
        '''
        update the obstacles in the environment

        '''

        self.pb_ompl_interface.set_obstacles(self.obstacles)
    

            
    def add_mesh(self,file_path,posi,ori):
        meshId = p.createCollisionShape(p.GEOM_MESH,fileName=file_path, meshScale=[1,1,1],flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        # create a multi body with the collision shape
        mesh_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=meshId, basePosition=posi, baseOrientation= ori )    
        self.obstacles.append(mesh_body)
        self.pb_ompl_interface.set_obstacles(self.obstacles)
        return mesh_body
    

    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        self.pb_ompl_interface.set_obstacles(self.obstacles)
        return box_id     
    



    # def continue_plan(self,path,goal):
    #     self.robot.set_state(path[-1])
    #     self.pb_ompl_interface.set_planner("BITstar")
    #     res, path = self.pb_ompl_interface.plan(goal,allowed_time=1)
    #     self.path = path


    def run(self, start, goal):
        self.robot.set_state(start)
        res, path = self.pb_ompl_interface.plan(goal)
        self.path = path
        print(path[-1])
        # thread1 = threading.Thread(target=self.pb_ompl_interface.execute, args=(path,))
        # thread2 = threading.Thread(target=self.continue_plan, args=(path, goal))

        # if res and path[-1] != goal:
        #     thread1.start()
        #     thread2.start()

        #     thread2.join()
        #     thread1.join()

        # while True:
        self.pb_ompl_interface.execute(self.path)
            
        return res, path


if __name__== '__main__':
        # ####  put mesh 
    ur5_move = myur5("robot1")
    env = UR5Demo([-0.2445,-0.6305,0.1093],[ 0.71562555 ,0.54186322 ,0.248717 , 0.36387385])
    # env = UR5Demo([-0.6239,-0.078,0.1805],[ 0.92099033,  0.02364499 ,-0.00190007 , 0.38886259])

    offset  = [-np.pi/2,0 ,0, 0  ,0  ,0]
    start  = ur5_move.get_current_joint()

    # goal_point = [0.1,-0.15,-0.2]
    # goal = p.calculateInverseKinematics(env.robot.id, 6, goal_point)

    end_point = [-0.3,-0.1,-0.2]
    goal = p.calculateInverseKinematics(env.robot.id, 6, end_point)

    env.robot.set_state(start)
  
    # load mesh
    # pcd = o3d.io.read_point_cloud("plydoc/bunny.ply")
    # env.add_mesh(pcd,to_bounding_box=True)
    env.add_mesh("plydoc/mesh13.obj",[0,0,0],[0,0,0,1])
    # env.add_box([1, 0, 0.7], [0.5, 0.5, 0.05])

    # res,pth = env.run(start,goal)
    input("Press Enter to continue...")
    ur5_move.move_(pth)
    



