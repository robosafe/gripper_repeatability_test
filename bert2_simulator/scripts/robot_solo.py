#!/usr/bin/env python
"""
Test script for robot movements on actual BERT2 hardware.
The left_wrist_pronation_joint is misbehaving (sensing issue?), so this script avoids using it.

Format of the vector of joint commands for Gazebo-ROS:
'hipRotor', 'hipFlexor', 'neckFlexor', 'neckRotor', 'leftShoulderFlexor', 'rightShoulderFlexor', 'leftShoulderAbduction',  'leftHumeralRotation', 'leftElbowFlexor','leftWristPronation', 'leftWristAbduction', 'leftWristFlexor'

Format of the vector of joint commands for the hand:
'leftThumb1', 'leftThumb2', 'leftIndex1', 'leftIndex2', 'leftMiddle1', 'leftMiddle2', 'leftAnular1', 'leftAnular2', 'leftLittle1', 'leftLittle2'

Format of the joints used for planning:
'hipRotor', 'hipFlexor', 'leftShoulderFlexor', 'leftShoulderAbduction',  'leftHumeralRotation', 'leftElbowFlexor','leftWristPronation', 'leftWristAbduction', 'leftWristFlexor'

Written by David Western, Feb 2016.
Derived from robot.py, by Dejanira Araiza-Illan.
"""

import os
import rospy
import smach
import smach_ros
import math
import random
import time
from robot_g import set_robot_joints
from robot_g import move_hand
from interface_plan import interface #Script that has the Moveit interface for planning
from bert2_simulator.msg import *
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from geometry_msgs.msg import Point
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
import bert2_interface

#--------------------------------------------------------------------------------------------------------------------
def Reset():


	move_hand('close')

        # Hand raised:
	targ = [0.5,0.05,-0.77,0.0,0.0,1.4,0.0,-0.17,0.0]
        targ = fixTarget(targ)
	theplans = interface(targ)

        followPlans(theplans)
        rospy.sleep(4)




#--------------------------------------------------------------------------------------------------------------------
def Move():

        # Move hand laterally to a point above the piece
        targ = [1.5,0.05,-0.77,0.0,0.0,1.4,0.0,-0.17,1000.0]   
        targ = fixTarget(targ)
	theplans = interface(targ)

        followPlans(theplans)
        move_hand('open')
        rospy.sleep(2)

        # Bring the hand down to the piece
	targ = [1.5,0.05,-0.67,1000.0,0,1.4,0.0,-0.17,1000.0]
        targ = fixTarget(targ)
	theplans = interface(targ)
        followPlans(theplans)
        rospy.sleep(3)

        # Close the hand
	move_hand('close')
	rospy.sleep(1)

        # Raise the hand
        targ = [1.5,0.05,-0.77,0.0,0.0,1.4,0.0,-0.17,1000.0]   
        targ = fixTarget(targ)
	theplans = interface(targ)
        followPlans(theplans)
	rospy.sleep(1)

	# Path planning towards goal location
	targ = [0.0,0.05,-0.77,0.0,0.0,1.4,0.0,-0.17,0.0]
        targ = fixTarget(targ)
	theplans = interface(targ)
        followPlans(theplans)

	rospy.sleep(5)

	
def fixTarget(target):
        fixes = [-13.7835, 0.0, 2.9655, 45.0000, 1.7611, -43.6531, 0.0, 6.3450, 1.3932]
        for k in range(0,len(fixes)-1):
                if target[k]!=1000.0:
                        target[k] += math.radians(fixes[k])
        return target

def followPlans(theplans):
	for i,plan in enumerate(theplans):
                del plan[5]  # Ignore right shoulder 
                positions = dict(zip(bert2.JOINT_NAMES, plan))
    
                bert2.set_joint_positions(positions)
        	rospy.sleep(0.11)
         
	
#-------------------------------------------------------------------------------------------------------
def Release():
    	move_hand('open')
	rospy.sleep(1)
	

#--------------------------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------------------------
def main():
        global is_a_sim
        is_a_sim = 1
        rospy.set_param('is_this_a_simulation',is_a_sim)

	rospy.init_node('robot', anonymous=True) #Start node first

        if is_a_sim:
            set_robot_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Make sure some joint angles have been published to bert2/joint_states before
        # instantiating bert2_interface.Body().
        global bert2
        bert2 = bert2_interface.Body()
        
        setmodel = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)	
        for k in range(0,10):        
                Reset()
                Move()
                Release()
                # Reset object
                rospy.wait_for_service('/gazebo/set_model_state')
	        setmodel(ModelState('object',Pose(Point(0.3,-0.44,1.2),Quaternion(0.0,0.0,0.0,1.0)),Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)),'world'))
                

        
	

#---------------------------------------------------------------------------------------------------
if __name__ == '__main__':
	
	try:
    		main()
	except rospy.ROSInterruptException: #to stop the code when pressing Ctr+c
        	pass
