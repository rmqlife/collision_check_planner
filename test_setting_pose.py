from myObsPlanner import *
import time
from copy import deepcopy

joint_configs = MyConfig('/home/rmqlife/work/ur_slam/slam_data/joint_configs.json')

def init_real_robots():
    rospy.init_node('dual_arm_bullet')
    real_robots = list()
    for ns in ['robot1', 'robot2']:
        real_robots.append( MyRobotNs(ns=ns) )
    return real_robots

def sync_from_real(planner, real_robots):
    return planner.set_joints([r.get_joints() for r in real_robots])

def init_env(test_on_real_robot=False):
    planner = MyObsPlanner(obj_path="plydoc/output_mesh.obj", pose_path='pose.json', planner_name="RRTConnect")
    # planner.add_box([-0.6,0.2,0],[0.1,0.1,0.5])    
    # planner.add_box([0,0.2,-0.5],[0.7,0.7,0.1])      
    if test_on_real_robot:
        real_robots = init_real_robots()
        sync_from_real(planner, real_robots)
        return planner, real_robots
    
    else:
        planner.set_joints([joint_configs.get('facedown'), joint_configs.get('facedown2')])
        return planner, None
    
if __name__ == "__main__":
    
    planner, real_robots =init_env(test_on_real_robot=True)
    
    if False:
        init_joints = planner.get_joints()
        poses = planner.get_poses()
        poses[0] = deepcopy(poses[1])
        # poses[0].t[1] -= 0.6
        planner.set_poses(poses, q_list=init_joints)
        joints = planner.get_joints()
        planner.set_joints(init_joints)

    joints = [joint_configs.get('facedown'), joint_configs.get('facedown2')]
    res, path_list = planner.plan(joints)


    print(f'Found solution: {res}, each path length: {len(path_list[0])}, {len(path_list[1])}')
    for _ in range(2):
        planner.run(path_list=path_list, sleep_time=1.0/20)

    input("test on real robot?")
    for step, joint in enumerate(zip(path_list[0], path_list[1])):
        for i, r in enumerate(real_robots):
            r.move_joints_smooth(joint[i], wait=(step==0))