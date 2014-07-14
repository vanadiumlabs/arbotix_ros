#!/usr/bin/env python

"""
  parallel_gripper_controller.py - controls a gripper built of two servos
  Copyright (c) 2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS' AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy, actionlib
import thread

from control_msgs.msg import GripperCommandAction
from std_msgs.msg import Float64
from math import asin

class ParallelGripperActionController:
    """ A simple controller that operates two opposing servos to
        open/close to a particular size opening. """
    def __init__(self):
        rospy.init_node('gripper_controller')
        rospy.logwarn("parallel_gripper_action_controller.py is deprecated and will be removed in ROS Indigo, please use gripper_controller")

        # trapezoid model: base width connecting each gripper's rotation point
            #              + length of gripper fingers to computation point
            #              = compute angles based on a desired width at comp. point
        self.pad_width = rospy.get_param('~pad_width', 0.01)
        self.finger_length = rospy.get_param('~finger_length', 0.02)
        self.min_opening = rospy.get_param('~min', 0.0)
        self.max_opening = rospy.get_param('~max', 2*self.finger_length)

        self.center_l = rospy.get_param('~center_left', 0.0)
        self.center_r = rospy.get_param('~center_right', 0.0)
        self.invert_l = rospy.get_param('~invert_left', False)
        self.invert_r = rospy.get_param('~invert_right', False)

        # publishers
        self.l_pub = rospy.Publisher('l_gripper_joint/command', Float64, queue_size=5)
        self.r_pub = rospy.Publisher('r_gripper_joint/command', Float64, queue_size=5)

        # subscribe to command and then spin
        self.server = actionlib.SimpleActionServer('~gripper_action', GripperCommandAction, execute_cb=self.actionCb, auto_start=False)
        self.server.start()
        rospy.spin()

    def actionCb(self, goal):
        """ Take an input command of width to open gripper. """
        rospy.loginfo('Gripper controller action goal recieved:%f' % goal.command.position)
        command = goal.command.position
        # check limits
        if command > self.max_opening:
            command = self.max_opening
        if command < self.min_opening:
            command = self.min_opening
        # compute angles
        angle = asin((command - self.pad_width)/(2*self.finger_length))
        if self.invert_l:
            l = -angle + self.center_l
        else:
            l = angle + self.center_l
        if self.invert_r:
            r = angle + self.center_r
        else:
            r = -angle + self.center_r
        # publish msgs
        lmsg = Float64(l)
        rmsg = Float64(r)
        self.l_pub.publish(lmsg)
        self.r_pub.publish(rmsg)
        rospy.sleep(5.0)
        self.server.set_succeeded()
        rospy.loginfo('Gripper Controller: Done.')

if __name__=='__main__': 
    try:
        ParallelGripperActionController()
    except rospy.ROSInterruptException:
        rospy.loginfo('Hasta la Vista...')
        
