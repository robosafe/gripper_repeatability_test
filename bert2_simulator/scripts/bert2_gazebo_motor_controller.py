#!/usr/bin/env python
## Pick up commands from the joint_trajectory_action_server
## and distribute them to the individual joint controllers for the
## Gazebo model of BERT2.
##
## Created by David Western, March 2015.

import rospy
#from std_msgs.msg import String

import bert2_core_msgs.msg
import bert2_interface
import std_msgs.msg
import sensor_msgs.msg
import moveit_msgs.msg

class BERT2GazeboMotorController:
    def __init__(self):

        self.jointPubs = {} # Dictionary of publisher for each joint.
                            # Will be populated as needed.

        joint_command_sub = rospy.Subscriber(
            '/bert2/joint_command',
            bert2_core_msgs.msg.JointCommand,
            self._on_joint_command,
            queue_size=1,
            tcp_nodelay=True)


    #-----------------------------------------------------------------------------------------------
    def _on_joint_command(self,msg):
	
        # Publish a new set point for each joint:
        for idx, joint in enumerate(msg.names):

            # First check that we've set up the publisher:
            if joint not in self.jointPubs:
                self._add_pub(joint)

            self.jointPubs[joint].publish(std_msgs.msg.Float64(msg.angles[idx]))


    #-----------------------------------------------------------------------------------------------
    def _add_pub(self,joint):
        # Add a publisher for this joint:

        # The topic will be inferred from the joint name.  Make sure
        # that the loaded controllers (in joint_controllers.yaml,
        # perhaps?) comply with the assumed naming structure.
        topic = '/bert2/'+joint+'_posControlr/command'
          
        print('Adding topic: '+topic)

        pub = rospy.Publisher(
            topic,
            std_msgs.msg.Float64,
            latch=True)
        
        self.jointPubs[joint] = pub




#---------------------------------------------------------------------------------------------------
def tempFixes():

    ## Echo /bert2/joint_states to /joint_states, where the move_group node expects to read joint states.
    ## As suggested at https://groups.google.com/a/rethinkrobotics.com/forum/#!msg/brr-users/P890sqFxpBo/7DSF_cOcUC8J
    ## it should be possible to just remap /joint_states to /bert2/joint_states when launching the
    ## move_group node, but that doesn't seem to work at the moment.  This workaround does the trick,
    ## but it means all the joint_states are getting published twice as much as they need to be.
    print( "    Echoing /bert2/joint_states to /joint_states for move_group node... " )
    p = rospy.Publisher(
        '/joint_states',
        sensor_msgs.msg.JointState,
        latch=True)
    s = rospy.Subscriber(
        '/bert2/joint_states',
        sensor_msgs.msg.JointState,
        p.publish,
        queue_size=3)
    """
    collnPub = rospy.Publisher(
        '/collision_object',
        moveit_msgs.msg.CollisionObject)
    cylinder_object = moveit_msgs.msg.CollisionObject
    """


#---------------------------------------------------------------------------------------------------
def main():
    
    print( "Initializing node... " )
    rospy.init_node('BERT2_gazebo_motor_controller')

    print( "Implementing some temporary fixes.  If you're looking to improve performance, you might start here.")
    tempFixes()

    print( "Initializing BERT2_gazebo_motor_controller..." )                              
    BGMC = BERT2GazeboMotorController()

    print( "Running. Ctrl-c to quit" )
    rospy.spin()

#---------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
