import sys
sys.path.append('/home/rmqlife/work/catkin_ur5/src/teleop/src')
sys.path.append('/home/rmqlife/work/collision_check/ompl/py-bindings')
import pybullet as p
import pybullet_data
import pb_ompl2
# from move_ur5 import *
import threading
'''
export PYTHONPATH=$PYTHONPATH:/home/rmqlife/work/collision_check/ompl/py-bindings 
'''


class UR5DualPlanner():
    def __init__(self,arm1_position,arm2_position,arm_1_orientation,arm_2_orientation):
        '''
        args:
            arm1_position: list, [x,y,z]
            arm2_position: list, [x,y,z]
            arm_1_orientation: [x,y,z,w])
            arm_2_orientation: [x,y,z,w])
        
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
        p.resetBasePositionAndOrientation(robot_1, arm1_position, arm_1_orientation)
        p.resetBasePositionAndOrientation(robot_2, arm2_position, arm_2_orientation)


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
        
    
if __name__== '__main__':
    # left side
    arm_1_position =  [-0.6226 , -0.03173 , 0.1716 ]
    arm_1_orientation = [ 0.92061594 , 0.00544071 ,-0.00534512,  0.39039482]

    # Right side
    arm_2_position = [-0.5918252 ,  0.47034631  ,0.16780731]
    arm_2_orientation = [ 0.92490632  ,0.03584233, -0.02131 ,-0.37719442]



    env = UR5DualPlanner(arm_1_position,arm_2_position,arm_1_orientation,arm_2_orientation)  

    ur5_move1 = myur5("robot1")
    ur5_move2 = myur5("robot2")

    robot1 = env.robot1
    robot2 = env.robot2

    start1 = ur5_move1.get_joints()
    env.robot1.set_state(start1)
    start2 = ur5_move2.get_current_joint()
    env.robot2.set_state(start2)

    
    env.add_mesh("plydoc/mesh22.obj",[0,0,0],[0,0,0,1])



    goal1_point = [0.1,0.1,0.2]
    goal1 = p.calculateInverseKinematics(robot1.id, 6, goal1_point)

    goal2_point = [-0.3,-0.1,-0.3]
    goal2 = p.calculateInverseKinematics(robot2.id, 6, goal2_point)


    
    path1,path2 = env.run(start1,goal1,start2,goal2)

    input("Press Enter to start...")
    thread1 = threading.Thread(target=ur5_move1.move_, args=(path1,))
    thread2 = threading.Thread(target=ur5_move2.move_, args=(path2,))
    thread1.start()
    thread2.start()



# 