from myObsPlanner import *
from test_real_robot_planning import init_real_robots, sync_from_real
from copy import deepcopy
test_on_real_robot = False

if __name__ == "__main__":
    planner = MyObsPlanner(obj_path="plydoc/mesh22.obj", pose_path='pose.json')
    joint_configs = MyConfig('/home/rmqlife/work/ur_slam/slam_data/joint_configs.json')

    if test_on_real_robot:
        rospy.init_node('dual_arm_bullet')
        real_robots = init_real_robots()
        sync_from_real(planner, real_robots)
        pass
    else:
        planner.set_joints([joint_configs.get('facedown'), joint_configs.get('facedown2')])


    # pose_list = planner.get_poses()
    # pose_list[0].t[0] -= 0.1
    # pose_list[1] = deepcopy(pose_list[0])
    # pose_list[1].t[1] += 0.4

    # planner.set_poses(pose_list=pose_list, q_list=planner.get_joints(), is_se3=True)





    path_list = list()
    for i in [0,1]:
        init_pose = SE3_to_pose(planner.get_poses()[i])
        target_pose = init_pose.copy()
        target_pose[2] -= 0.3
        poses = circle_pose(init_pose, target_pose[:3], radius=0.1, num_points=50)
        path = planner.robot_ik[i].plan_trajectory(poses, planner.get_joints()[i])
        #
        fixed_val = planner.get_joints()[i][5]
        for i, j in enumerate(path):
            path[i][5] = fixed_val
        path_list.append(path)




    while 1:
        for j0, j1 in zip(path_list[0], path_list[1]):
            planner.set_joints([j0, j1])
            import time
            time.sleep(1./60.)
        
        if test_on_real_robot:
            break

    input("test on real robot?")
    # keep the 6th joints as fixed


    for step, joint in enumerate(zip(path_list[0], path_list[1])):
        for i, r in enumerate(real_robots):
            r.move_joints_smooth(joint[i], wait=(step==0))
