#!/usr/bin/env python
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import *
import math
import rospy
# from utils import *

__REQUIRED_API_VERSION__ = "1"  # API version
__ROBOT_VELOCITY__ = 0.5        # velocity of the robot

# main program
def start_program():
    # print the current position of thr robot in the terminal
    rospy.loginfo("Current pose: %s", r.get_current_pose(target_link="Tip", base="base_link")) 

    # move to starting position
    # rospy.loginfo("Move to starting position 1 ") 
    # r.move(Ptp(goal=[0.0 , 1.65 , 0.4 , 0 , 1 , 3.65], 
    #     planning_group="denso_arm_torch", reference_frame="base_link",  
    #     vel_scale=0.4, acc_scale=0.4))
    # rospy.loginfo("\tNew pose: %s", r.get_current_pose(target_link="Tip", base="base_link")) 

    rospy.loginfo("Move to starting point 1") 
    r.move(Ptp(goal=Pose(position=Point(-0.5028, 0.0000, 0.5386), orientation=Quaternion(0.0135, 0.9238, 0.0056, -0.3827)), 
           relative=False, reference_frame="base_link", planning_group="denso_arm_torch", target_link="Tip",
           vel_scale=0.5, acc_scale=0.5))
    rospy.loginfo("\tNew pose: %s", r.get_current_pose(target_link="Tip", base="base_link"))

    rospy.loginfo("Move to starting point 2") 
    r.move(Lin(goal=Pose(position=Point(-0.5325, 0.0019, 0.0340), orientation=Quaternion(0.0134, 0.9238, 0.0055, -0.3827)), 
           relative=False, reference_frame="base_link", planning_group="denso_arm_torch", target_link="Tip",
           vel_scale=0.1, acc_scale=0.1))
    rospy.loginfo("\tNew pose: %s", r.get_current_pose(target_link="Tip", base="base_link"))

    # Welding trajectory 1:
    #   1. Move left & change orientation
    #   2. Move Forward

    rospy.loginfo("Move left and change orientation") 
    r.move(Lin(goal=Pose(position=Point(-0.5298, 0.0946, 0.0340), orientation=Quaternion(-0.4009, 0.8119, -0.1325, -0.4031)), 
           relative=False, reference_frame="base_link", planning_group="denso_arm_torch", target_link="Tip",
           vel_scale=0.05, acc_scale=0.05))
    rospy.loginfo("\tNew pose: %s", r.get_current_pose(target_link="Tip", base="base_link"))

#     rospy.loginfo("Move forward") 
#     r.move(Lin(goal=Pose(position=Point(0.012, 0.0, 0)), 
#            relative=True, reference_frame="BASE", planning_group="denso_arm_torch", target_link="Tip",
#            vel_scale=0.1, acc_scale=0.1))
#     rospy.loginfo("\tNew pose: %s", r.get_current_pose(target_link="Tip", base="base_link"))

    # Welding trajectory 2:
    #   1. Move left 
    #   2. Circular motion

    # rospy.loginfo("Move left and change orientation") 
    # r.move(Lin(goal=Pose(position=Point(0, -0.2, 0), orientation=from_rpy(0,0,0)), 
    #        relative=True, reference_frame="BASE", planning_group="denso_arm_torch", target_link="Tip",
    #        vel_scale=0.1, acc_scale=0.1))
    # rospy.loginfo("\tNew pose: %s", r.get_current_pose(target_link="Tip", base="base_link"))

    # rospy.loginfo("Move circular") 
    # r.move(Circ(goal=Pose(position=Point(-0.632, -0.107, 0.132)), center=Point(-0.632, -0.095, 0.132),
    #         reference_frame="base_link", planning_group="denso_arm_torch", target_link="Tip",
    #         vel_scale=0.1, acc_scale=0.1))
    # rospy.loginfo("\tNew pose: %s", r.get_current_pose(target_link="Tip", base="base_link"))

    # Welding trajectory 3:
    #   1. Move left & change orientation
    #   2. Blend
    #   3. Move Forward

    # Welding trajectory 4:
    #   1. Move left 
    #   2. Blend
    #   3. Circular motion

if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    start_program()