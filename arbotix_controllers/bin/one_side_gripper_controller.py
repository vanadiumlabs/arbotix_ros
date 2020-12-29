#!/usr/bin/env python3

"""
  one_side_gripper_controller.py - controls a gripper built with one servo
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
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
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

import rospy
import thread

from std_msgs.msg import Float64
from math import asin

class OneSideGripperController:
    """ A simple controller that operates a servos to
        open/close to a particular size opening. """
    def __init__(self):
        rospy.init_node("one_side_gripper_controller")
        rospy.logwarn("one_side_gripper_controller.py is deprecated and will be removed in ROS Indigo, please use gripper_controller")

        self.pad_width = rospy.get_param("~pad_width", 0.01)
        self.finger_length = rospy.get_param("~finger_length", 0.02)
        self.center = rospy.get_param("~center", 0.0)
        self.invert = rospy.get_param("~invert", False)

        # publishers
        self.pub = rospy.Publisher("gripper_joint/command", Float64, queue_size=5)

        # subscribe to command and then spin
        rospy.Subscriber("~command", Float64, self.commandCb)
        rospy.spin()

    def commandCb(self, msg):
        """ Take an input command of width to open gripper. """
        # check limits
        #if msg.data > self.max_opening or msg.data < self.min_opening:
        #    rospy.logerr("Command exceeds limits.")
        #    return
        # compute angle
        angle = asin((msg.data - self.pad_width)/(2*self.finger_length))
        # publish message
        if self.invert:
            self.pub.publish(-angle + self.center)
        else:
            self.pub.publish(angle + self.center)

if __name__=="__main__":
    try:
        OneSideGripperController()
    except rospy.ROSInterruptException:
        rospy.loginfo("Hasta la Vista...")
        
