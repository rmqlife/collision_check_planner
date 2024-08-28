import pybullet as p
import pybullet_data
import pb_ompl
import time
import threading
import open3d as o3d
from pointcloud_util import pointcloud

class DualUR5():
    def __init__(self):
        self.obstacles1 = []  # obstacles for robot1
        self.obstacles2 = []  # obstacles for robot2

        # Initialize PyBullet environment
        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Add a ground plane as an obstacle
        plane = p.loadURDF("plane.urdf")
        self.obstacles1.append(plane)

        # Load UR5e robots
        urdf_path = "ur5e/ur5e.urdf"
        self.robot_1 = p.loadURDF(urdf_path, (0, 0, 0), useFixedBase=1)
        self.robot_2 = p.loadURDF(urdf_path, (0.5, 0.5, 0), useFixedBase=1)

        # Initialize PbOMPLRobot instances
        self.robot1 = pb_ompl.PbOMPLRobot(self.robot_1)
        self.robot2 = pb_ompl.PbOMPLRobot(self.robot_2)

        # Initialize PbOMPL interfaces
        self.pb_ompl_interface1 = pb_ompl.PbOMPL(self.robot1, self.obstacles1)
        self.pb_ompl_interface2 = pb_ompl.PbOMPL(self.robot2, self.obstacles2)

        # Set planner for both interfaces
        self.pb_ompl_interface1.set_planner("BITstar")
        self.pb_ompl_interface2.set_planner("BITstar")

        # Initialize paths for robots
        self.path1 = []
        self.path2 = []

    def clear_obstacles(self):
        for obstacle in self.obstacles1:
            p.removeBody(obstacle)
        for obstacle in self.obstacles2:
            p.removeBody(obstacle)

    def update_obstacles(self):
        self.pb_ompl_interface1.set_obstacles(self.obstacles1)
        self.pb_ompl_interface2.set_obstacles(self.obstacles2)

    def add_mesh(self, pcd, to_bounding_box=False):
        # Import mesh as PyBullet collision shape
        pointcloud_obj = pointcloud(pcd)
        mesh = pointcloud_obj.create_mesh()
        meshFile = pointcloud_obj.export(mesh, "plydoc/model_{time.strftime(\"%m%d-%H%M\")}.obj")
        pos = pointcloud_obj.get_pos()

        # print("mesh pos is !!!!!", pos)
        if not to_bounding_box:
            meshId = p.createCollisionShape(p.GEOM_MESH, fileName=meshFile, meshScale=[1, 1, 1])
            mesh_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=meshId, basePosition=pos)
            self.obstacles1.append(mesh_body)
            self.obstacles2.append(mesh_body)
        else:
            mesh = pointcloud_obj.get_mesh()
            min_x, min_y, min_z, max_x, max_y, max_z = pointcloud_obj.to_bounding_box(mesh)
            collision_box_id = p.createCollisionShape(p.GEOM_BOX,
                                                      halfExtents=[(max_x - min_x) / 2,
                                                                   (max_y - min_y) / 2,
                                                                   (max_z - min_z) / 2],
                                                      meshScale=[1, 1, 1])
            coll_box = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_box_id, basePosition=pos)
            self.obstacles1.append(coll_box)
            self.obstacles2.append(coll_box)

    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)
        self.obstacles1.append(box_id)
        self.obstacles2.append(box_id)
        return box_id

    def control_robot1(self, joint_positions_list):
        for joint_positions in joint_positions_list:
            for _ in range(1):
                for joint_index, position in enumerate(joint_positions):
                    p.setJointMotorControl2(self.robot_1, jointIndex=joint_index, controlMode=p.POSITION_CONTROL, targetPosition=position)
                p.stepSimulation()
            time.sleep(1./240.)

    def control_robot2(self, joint_positions_list):
        for joint_positions in joint_positions_list:
            for _ in range(1):
                for joint_index, position in enumerate(joint_positions):
                    p.setJointMotorControl2(self.robot_2, jointIndex=joint_index, controlMode=p.POSITION_CONTROL, targetPosition=position)
                p.stepSimulation()
            time.sleep(1./240.)

    def run(self, start1, goal1, start2, goal2):
        self.robot1.set_state(start1)
        self.robot2.set_state(start2)

        res1, path1 = self.pb_ompl_interface1.plan(goal1)
        res2, path2 = self.pb_ompl_interface2.plan(goal2)

        print(path1[-1],path2[-1])


        # Create threads for controlling robots
        thread1 = threading.Thread(target=self.control_robot1, args=(path1,))
        thread2 = threading.Thread(target=self.control_robot2, args=(path2,))
        
        # Start threads
        thread1.start()
        thread2.start()

        # Wait for threads to complete
        thread1.join()
        thread2.join()

if __name__ == '__main__':
    env = DualUR5()
    start = [0, -2, 0, -1, 0, 0]
    goal = [0, 0, 0, 0, 0, -1.2]
    # env.add_box([1, 0, 0.7], [0.5, 0.5, 0.05])
    path = "mesh/screen.ply"
    point_cloud = o3d.io.read_point_cloud(path)
    env.add_mesh(point_cloud, to_bounding_box=False)
    env.run(start, goal, start, goal)
    while True:
        a=1
