from myRobot import MyRobotNs
import numpy as np
import rospy

class myur5(MyRobotNs):
    def __init__(self,Ns):
        try:
            rospy.init_node(Ns, anonymous=True)
        except:
            print("node already exist")
        super().__init__(Ns)

    def get_current_joint(self):
        return super().get_joints()
    
    def move_(self,path):
        # Initialize the UR5 robot
        
        #check initial position
        current_joint = self.get_current_joint()
        print("start joint", current_joint)

        if (max(abs(path[0]-current_joint))>0.3):
            print("start joint is not correct")
        else:
            step = 0
            for joint_path in path:
                diff  = [abs(a-b) for a,b in zip(joint_path,current_joint)]
                joints_movement = np.max(diff)
                self.move_joints(joint_path,duration =0.5* joints_movement, wait = True)
                current_joint = joint_path
                # step+=1
                # if step%100 ==0:
                #     input("press enter to continue")
                

