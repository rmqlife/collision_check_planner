from myObsPlanner import *
import time

def init_real_robots():
    rospy.init_node('dual_arm_bullet')
    real_robots = list()
    for ns in ['robot1', 'robot2']:
        real_robots.append( MyRobotNs(ns=ns) )
    return real_robots

def sync_from_real(planner, real_robots):
    return planner.set_joints([r.get_joints() for r in real_robots])



if __name__ == "__main__":
    planner = MyObsPlanner(obj_path="plydoc/mesh22.obj", pose_path='pose.json', planner_name="RRTConnect")
    planner.add_box([-0.6,0.2,0],[0.1,0.1,0.5])    
    planner.add_box([0,0.2,-0.5],[0.7,1,0.1])    

    joint_configs = MyConfig('/home/rmqlife/work/ur_slam/slam_data/joint_configs.json')
    
    real_robots = init_real_robots()
    sync_from_real(planner=planner, real_robots=real_robots)

    poses = planner.get_poses()
    print("environment pose")
    for se3_pose in poses:
        se3_pose.printline()
    
    if True:
        res, path_list = planner.plan([joint_configs.get('facedown'), joint_configs.get('facedown2')])

        print(f'Found solution: {res}, each path length: {len(path_list[0])}, {len(path_list[1])}')

        while True:
            sync_from_real(planner=planner, real_robots=real_robots)
            for j0, j1 in zip(path_list[0], path_list[1]):
                planner.set_joints([j0, j1])
                time.sleep(1./60.)
    # planner.plan([planner.get_joints()[1]])
    input("break")
