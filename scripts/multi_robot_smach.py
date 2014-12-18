#! /usr/bin/env python
#*********************************************************************
# Software License Agreement (BSD License)
#
#  Copyright (c) 2011 Andrew J. Harris
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#********************************************************************/

import roslib; roslib.load_manifest('smach_usecase')
import rospy
import smach
import smach_ros

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from std_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *

# main
def main():
    rospy.init_node('multi_robot_smach_state_machine')

    #
    # Define the top level state machine.  It
    # only has two transitions out, aborted and
    # preempted.
    #
    sm = smach.StateMachine(['aborted','preempted'])
    with sm:
        # 
        # Define a navigation goal that will put the robot just 
        # outside room 1.
        # 
        robot1_init_goal = MoveBaseGoal(target_pose = 
                                   PoseStamped(header = 
                                               Header(frame_id = '/map'),
                                               pose = 
                                               Pose(position = Point(x = -2.0, y = -0.8, z = 0.0),
                                                    orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0))))

        #
        # Add a new state called "GOTO_HALL_1" to the state machine
        # and when the state is entered, send the hall_1_goal action
        # request to move_base.
        #
        smach.StateMachine.add('GOTO_INIT_1',
                               smach_ros.SimpleActionState('/robot1/move_base',
                                                           MoveBaseAction,
                                                           goal=robot1_init_goal),
                               transitions={'succeeded':'GOTO_INIT_2'})


        # 
        # Define a navigation goal that will put the robot just 
        # in front of artifact 1.
        # 
        robot2_init_goal = MoveBaseGoal(target_pose = 
                                       PoseStamped(header = 
                                                   Header(frame_id = '/map'),
                                                   pose = 
                                                   Pose(position = Point(x = -0.5, y = -0.8, z = 0.0),
                                                        orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0))))

        smach.StateMachine.add('GOTO_INIT_2',
                               smach_ros.SimpleActionState('/robot2/move_base',
                                                           MoveBaseAction,
                                                           goal=robot2_init_goal),
                               transitions={'succeeded':'ROBOT1_2_ROBOT2'})


        robot1_2_robot2_goal = MoveBaseGoal(target_pose = 
                                       PoseStamped(header = 
                                                   Header(frame_id = '/map'),
                                                   pose = 
                                                   Pose(position = Point(x = -0.8, y = -0.8, z = 0.0),
                                                        orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0))))

        smach.StateMachine.add('ROBOT1_2_ROBOT2',
                               smach_ros.SimpleActionState('/robot1/move_base',
                                                           MoveBaseAction,
                                                           goal=robot1_2_robot2_goal),
                               transitions={'succeeded':'ROBOT2_2_END'})
			       
			       
        robot2_2_end_goal = MoveBaseGoal(target_pose = 
                                       PoseStamped(header = 
                                                   Header(frame_id = '/map'),
                                                   pose = 
                                                   Pose(position = Point(x = 0.8, y = 0.8, z = 0.0),
                                                        orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0))))

        smach.StateMachine.add('ROBOT2_2_END',
                               smach_ros.SimpleActionState('/robot2/move_base',
                                                           MoveBaseAction,
                                                           goal=robot2_2_end_goal),
                               transitions={'succeeded':'preempted',
                                            'aborted':'aborted',
                                            'preempted':'preempted'})			       
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('guard', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
