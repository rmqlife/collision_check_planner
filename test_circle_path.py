from myObsPlanner import *
from test_setting_pose import init_env,  joint_configs
from copy import deepcopy

test_on_real_robot = False

if __name__ == "__main__":
    planner, real_robots =init_env(test_on_real_robot)

    path_list = list()
    for i in [0,1]:
        init_pose = SE3_to_pose(planner.get_poses()[i])
        target_pose = deepcopy(init_pose)
        target_pose[2] -= 0.3
        poses = circle_pose(init_pose, target_pose[:3], radius=0.1, num_points=50)
        poses.append(init_pose)
        path = planner.robot_ik[i].plan_trajectory(poses, planner.get_joints()[i])
        
        # fixed 6th joint 
        fixed_val = planner.get_joints()[i][5]
        for i, j in enumerate(path):
            path[i][5] = fixed_val
        path_list.append(path)

    while 1:
        planner.run(path_list)
        if test_on_real_robot:
            break

    input("test on real robot?")
    # keep the 6th joints as fixed


    for step, joint in enumerate(zip(path_list[0], path_list[1])):
        for i, r in enumerate(real_robots):
            r.move_joints_smooth(joint[i], wait=(step==0))
