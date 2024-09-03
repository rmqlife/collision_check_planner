try:

    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'ompl/py-bindings'))
    # sys.path.insert(0, join(dirname(abspath(__file__)), '../whole-body-motion-planning/src/ompl/py-bindings'))
    
    print(sys.path)
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
import pybullet as p
import pb_utils
import time
from itertools import product
import threading
import numpy as np



INTERPOLATE_NUM = 20
DEFAULT_PLANNING_TIME = 10

class PbOMPLRobot():
    '''
    To use with Pb_OMPL. You need to construct a instance of this class and pass to PbOMPL.

    Note:
    This parent class by default assumes that all joints are acutated and should be planned. If this is not your desired
    behaviour, please write your own inheritated class that overrides respective functionalities.
    '''
    def __init__(self, id) -> None:
        # Public attributes
        self.id = id
        # prune fixed joints
        all_joint_num = p.getNumJoints(id)
        all_joint_idx = list(range(all_joint_num))
        joint_idx = [j for j in all_joint_idx if self._is_not_fixed(j)]
        self.num_dim = len(joint_idx)
        self.joint_idx = joint_idx
        print("!!! self.joint idx: ", self.joint_idx)
        self.joint_bounds = []

        self.reset()

    def _is_not_fixed(self, joint_idx):
        joint_info = p.getJointInfo(self.id, joint_idx)
        return joint_info[2] != p.JOINT_FIXED

    def get_joint_bounds(self):
        '''
        Get joint bounds.
        By default, read from pybullet
        '''
        for i, joint_id in enumerate(self.joint_idx):
            joint_info = p.getJointInfo(self.id, joint_id)
            low = joint_info[8] # low bounds
            high = joint_info[9] # high bounds
            if low < high:
                self.joint_bounds.append([low, high])
        print("Joint bounds: {}".format(self.joint_bounds))
        return self.joint_bounds

    def get_cur_state(self):
        '''
        
        '''
        p_state = p.getJointStates(self.id, self.joint_idx)
        cur_state = []
        for i in range(self.num_dim):
            cur_state.append(p_state[i][0])
        return cur_state
    
    def set_state(self, state):
        '''
        Set robot state.
        To faciliate collision checking
        Args:
            state: list[Float], joint values of robot
        '''
        self._set_joint_positions(self.joint_idx, state)
        self.state = state

    def reset(self):
        '''
        Reset robot state
        Args:
            state: list[Float], joint values of robot
        '''

        state = [0] * self.num_dim

        self._set_joint_positions(self.joint_idx, state)
        self.state = state

    def _set_joint_positions(self, joints, positions):
        # for joint, value in zip(joints, positions):
        #     p.resetJointState(self.id, joint, value, targetVelocity=0
        for i in range(len(joints)):
            p.resetJointState(self.id, joints[i], positions[i], targetVelocity=0)

class PbStateSpace(ob.RealVectorStateSpace):
    def __init__(self, num_dim) -> None:
        super().__init__(num_dim)
        self.num_dim = num_dim
        self.state_sampler = None

    def allocStateSampler(self):
        '''
        This will be called by the internal OMPL planner
        '''
        # WARN: This will cause problems if the underlying planner is multi-threaded!!!
        if self.state_sampler:
            return self.state_sampler

        # when ompl planner calls this, we will return our sampler
        return self.allocDefaultStateSampler()

    def set_state_sampler(self, state_sampler):
        '''
        Optional, Set custom state sampler.
        '''
        self.state_sampler = state_sampler

class PbOMPL2():
    def __init__(self, robot1, robot2, obstacles=None):
        '''
        Initialize PbOMPL2 with two robots and optional obstacles
        Args:
            robot1: PbOMPLRobot, first robot
            robot2: PbOMPLRobot, second robot
            obstacles: list, list of obstacle IDs
        '''
        self.robot1 = robot1
        self.robot2 = robot2
        self.obstacles = obstacles if obstacles is not None else []

        # Setup state spaces and planners for both robots
        self.setup_spaces_and_planners()

    def setup_spaces_and_planners(self):
        '''
        Setup a single RealVectorStateSpace for both robots
        '''
        # 合并两个 RealVectorStateSpace 为一个大的 RealVectorStateSpace
        total_dim = self.robot1.num_dim + self.robot2.num_dim
        self.combined_space = ob.RealVectorStateSpace(total_dim)
        
        # 设置联合边界
        bounds = ob.RealVectorBounds(total_dim)
        joint_bounds1 = self.robot1.get_joint_bounds()
        joint_bounds2 = self.robot2.get_joint_bounds()
        
        for i, bound in enumerate(joint_bounds1):
            bounds.setLow(i, bound[0])
            bounds.setHigh(i, bound[1])
        
        for i, bound in enumerate(joint_bounds2):
            bounds.setLow(self.robot1.num_dim + i, bound[0])
            bounds.setHigh(self.robot1.num_dim + i, bound[1])
        
        self.combined_space.setBounds(bounds)
        
        # 使用单一空间设置 SimpleSetup
        self.ss = og.SimpleSetup(self.combined_space)
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid_combined))
        self.si = self.ss.getSpaceInformation()
        self.si.setStateValidityCheckingResolution(0.01)


    def set_obstacles(self, obstacles):
        '''
        Set overall obstacles for the environment
        Args:
            obstacles: list of obstacle IDs
        '''
        self.obstacles = obstacles
        # Update collision detection
        self.setup_collision_detection()

    def add_obstacles(self, obstacle_id):
        '''
        Add a single obstacle to the environment
        '''
        self.obstacles.append(obstacle_id)

    def remove_obstacles(self, obstacle_id):
        '''
        Remove a single obstacle from the environment
        '''
        self.obstacles.remove(obstacle_id)

    def is_state_valid_combined(self, state):
        '''
        Check if the combined state is valid:
            1. Extract individual robot states
            2. Check collisions
        '''
        # 分离机器人状态
        state1 = [state[i] for i in range(self.robot1.num_dim)]
        state2 = [state[i + self.robot1.num_dim] for i in range(self.robot2.num_dim)]
        
        # 检查机器人1的碰撞
        self.robot1.set_state(state1)
        for link1, link2 in self.check_link_pairs1:
            if pb_utils.pairwise_link_collision(self.robot1.id, link1, self.robot1.id, link2):
                return False
        for body1, body2 in self.check_body_pairs1:
            if pb_utils.pairwise_collision(body1, body2):
                return False
        
        # 检查机器人2的碰撞
        self.robot2.set_state(state2)
        for link1, link2 in self.check_link_pairs2:
            if pb_utils.pairwise_link_collision(self.robot2.id, link1, self.robot2.id, link2):
                return False
        for body1, body2 in self.check_body_pairs2:
            if pb_utils.pairwise_collision(body1, body2):
                return False
        
        # 检查机器人1与机器人2之间的碰撞
        if pb_utils.body_collision(self.robot1.id, self.robot2.id):
            return False
        
        return True

    def setup_collision_detection(self, self_collisions=True, allow_collision_links=[]):
        '''
        Setup collision detection for the environment
            1. Check self-collision for both robots
            2. Check collision with obstacles for both robots
            3. Check collision between robot1 and robot2
        '''
        self.check_link_pairs1 = pb_utils.get_self_link_pairs(self.robot1.id, self.robot1.joint_idx)
        self.check_link_pairs2 = pb_utils.get_self_link_pairs(self.robot2.id, self.robot2.joint_idx)
        
        moving_links1 = frozenset([item for item in pb_utils.get_moving_links(self.robot1.id, self.robot1.joint_idx)])
        moving_links2 = frozenset([item for item in pb_utils.get_moving_links(self.robot2.id, self.robot2.joint_idx)])
        
        moving_bodies1 = [(self.robot1.id, moving_links1)]
        moving_bodies2 = [(self.robot2.id, moving_links2)]
        
        self.check_body_pairs1 = list(product(moving_bodies1, self.obstacles))
        self.check_body_pairs2 = list(product(moving_bodies2, self.obstacles))
        self.check_body_pairs3 = list(product(moving_bodies1, moving_bodies2))

    def set_planner(self, planner_name):
        '''
        Set planner for the combined space
        Args:
            planner_name: string, name of the planner to use
        '''
        if planner_name == "PRM":
            self.planner = og.PRM(self.ss.getSpaceInformation())
        elif planner_name == "RRT":
            self.planner = og.RRT(self.ss.getSpaceInformation())
        elif planner_name == "RRTConnect":
            self.planner = og.RRTConnect(self.ss.getSpaceInformation())
        elif planner_name == "RRTstar":
            self.planner = og.RRTstar(self.ss.getSpaceInformation())
        elif planner_name == "EST":
            self.planner = og.EST(self.ss.getSpaceInformation())
        elif planner_name == "FMT":
            self.planner = og.FMT(self.ss.getSpaceInformation())
        elif planner_name == "BITstar":
            self.planner = og.BITstar(self.ss.getSpaceInformation())
        elif planner_name == "LBTRRT":
            self.planner = og.LazyRRT(self.ss.getSpaceInformation())
        elif planner_name == "BFMT":
            self.planner = og.BFMT(self.ss.getSpaceInformation())
        elif planner_name == "SST":
            self.planner = og.SST(self.ss.getSpaceInformation())
        elif planner_name == "BiEST":
            self.planner = og.BiEST(self.ss.getSpaceInformation()) 
        elif planner_name == "InformedRRTstar":
            self.planner = og.InformedRRTstar(self.ss.getSpaceInformation())
        elif planner_name == "SORRTstar":
            self.planner = og.SORRTstar(self.ss.getSpaceInformation())
        elif planner_name == "PRMstar":
            self.planner = og.PRMstar(self.ss.getSpaceInformation())

        else:
            print("{} not recognized, please add it first".format(planner_name))
            return
      

        self.ss.setPlanner(self.planner)

    def plan_start_goal(self, start1, goal1, start2, goal2, allowed_time=DEFAULT_PLANNING_TIME):
        print("start_planning")

        orig_robot_state1 = self.robot1.get_cur_state()
        orig_robot_state2 = self.robot2.get_cur_state()

        # 设置联合的起始和目标状态
        s = ob.State(self.combined_space)
        g = ob.State(self.combined_space)

        # 设置机器人1的起始和目标状态
        for i in range(len(start1)):
            s[i] = start1[i]
            g[i] = goal1[i]
        
        # 设置机器人2的起始和目标状态
        for i in range(len(start2)):
            s[self.robot1.num_dim + i] = start2[i]
            g[self.robot1.num_dim + i] = goal2[i]
        
        self.ss.setStartAndGoalStates(s, g)

        # 尝试在允许的时间内解决问题
        solved = self.ss.solve(allowed_time)
        res = False
        sol_path_list1 = []
        sol_path_list2 = []
        if solved:
            print("Found solution: interpolating into {} segments".format(INTERPOLATE_NUM))
            self.ss.simplifySolution()
            sol_path_geometric = self.ss.getSolutionPath()
            sol_path_geometric.interpolate(INTERPOLATE_NUM)
            sol_path_states = sol_path_geometric.getStates()
            sol_path_list = [self.state_to_list(state) for state in sol_path_states]
            # 分离机器人1和机器人2的路径
            sol_path_list1 = [state[:self.robot1.num_dim] for state in sol_path_list]
            sol_path_list2 = [state[self.robot1.num_dim:] for state in sol_path_list]
            if sol_path_list1[-1] == goal1 and sol_path_list2[-1] == goal2:
                res = True
            else:
                res = False
    
        else:
            print("No solution found")

        # 重置机器人状态
        self.robot1.set_state(orig_robot_state1)
        self.robot2.set_state(orig_robot_state2)
        return res, sol_path_list1, sol_path_list2

    
    def plan(self, goal1, goal2, allowed_time=DEFAULT_PLANNING_TIME):
        '''
        Plan a path to goal from current robot states
        Args:
            goal1, goal2: goal states for robot1 and robot2
            allowed_time: time allowed for planning
        '''
        start1 = self.robot1.get_cur_state()
        start2 = self.robot2.get_cur_state()
    
        return self.plan_start_goal(start1, goal1, start2, goal2, allowed_time)

    def execute(self, path1, path2):
        '''
        Execute a planned plan. Will visualize in pybullet.
        Args:
            path1, path2: lists of states for robot1 and robot2
        '''
        def control_robot(robot, joint_positions):
            for _ in range(1):
                # Move joints to desired positions
                robot._set_joint_positions(robot.joint_idx, joint_positions)
                p.stepSimulation()
                time.sleep(1./240.)

        for num in range(len(path1)):
            # Create threads for each robot simulation
            thread1 = threading.Thread(target=control_robot, args=(self.robot1, path1[num]))
            thread1 = threading.Thread(target=control_robot,args=(self.robot1,path1[num]))
            thread2 = threading.Thread(target=control_robot,args=(self.robot2,path2[num]))
            # Start both threads
            thread1.start()
            thread2.start()
            thread1.join()
            thread2.join()
            

    def state_to_list(self, state):
        return [state[i] for i in range(self.robot1.num_dim+self.robot2.num_dim)]
    
    