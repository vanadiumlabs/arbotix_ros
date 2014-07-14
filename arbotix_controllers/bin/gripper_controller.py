#!/usr/bin/env python

"""
  gripper_controller - action based controller for grippers.
  Copyright (c) 2011-2014 Vanadium Labs LLC.  All right reserved.

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

import rospy, actionlib
import thread

from control_msgs.msg import GripperCommandAction
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from math import asin

class TrapezoidGripperModel:
    """ A simple gripper with two opposing servos to open/close non-parallel jaws. """

    def __init__(self):
        # trapezoid model: base width connecting each gripper's rotation point
            #              + length of gripper fingers to computation point
            #              = compute angles based on a desired width at comp. point
        self.pad_width = rospy.get_param('~pad_width', 0.01)
        self.finger_length = rospy.get_param('~finger_length', 0.02)
        self.min_opening = rospy.get_param('~min_opening', 0.0)
        self.max_opening = rospy.get_param('~max_opening', 0.09)
        self.center_l = rospy.get_param('~center_left', 0.0)
        self.center_r = rospy.get_param('~center_right', 0.0)
        self.invert_l = rospy.get_param('~invert_left', False)
        self.invert_r = rospy.get_param('~invert_right', False)

        self.left_joint = rospy.get_param('~joint_left', 'l_gripper_joint')
        self.right_joint = rospy.get_param('~joint_right', 'r_gripper_joint')

        # publishers
        self.l_pub = rospy.Publisher(self.left_joint+'/command', Float64, queue_size=5)
        self.r_pub = rospy.Publisher(self.right_joint+'/command', Float64, queue_size=5)

    def setCommand(self, command):
        # check limits
        if command.position > self.max_opening or command.position < self.min_opening:
            rospy.logerr("Command (%f) exceeds opening limits (%f, %f)",
                          command.position, self.max_opening, self.min_opening)
            return False
        # compute angles
        angle = asin((command.position - self.pad_width)/(2*self.finger_length))
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
        return True

    def getPosition(self, js):
        left = right = 0
        for i in range(len(js.name)):
            if js.name[i] == self.left_joint:
                left = js.position[i]
            elif js.name[i] == self.right_joint:
                right = js.position[i]
        # TODO

        return 0.0

    def getEffort(self, joint_states):
        return 1.0

class ParallelGripperModel:
    """ One servo to open/close parallel jaws, typically via linkage. """

    def __init__(self):
        self.center = rospy.get_param('~center', 0.0)
        self.scale = rospy.get_param('~scale', 1.0)
        self.joint = rospy.get_param('~joint', 'gripper_joint')

        # publishers
        self.pub = rospy.Publisher(self.joint+'/command', Float64, queue_size=5)

    def setCommand(self, command):
        self.pub.publish((command.position * self.scale) + self.center)

    def getPosition(self, joint_states):
        return 0.0

    def getEffort(self, joint_states):
        return 1.0


class OneSideGripperModel:
    """ Simplest of grippers, one servo opens or closes to achieve a particular size opening. """

    def __init__(self):
        self.pad_width = rospy.get_param('~pad_width', 0.01)
        self.finger_length = rospy.get_param('~finger_length', 0.02)
        self.min_opening = rospy.get_param('~min_opening', 0.0)
        self.max_opening = rospy.get_param('~max_opening', 0.09)
        self.center = rospy.get_param('~center', 0.0)
        self.invert = rospy.get_param('~invert', False)
        self.joint = rospy.get_param('~joint', 'gripper_joint')

        # publishers
        self.pub = rospy.Publisher(self.joint+'/command', Float64, queue_size=5)

    def setCommand(self, command):
        """ Take an input command of width to open gripper. """
        # check limits
        if command.position > self.max_opening or command.position < self.min_opening:
            rospy.logerr("Command (%f) exceeds opening limits (%f, %f)",
                          command.position, self.max_opening, self.min_opening)
            return False
        # compute angle
        angle = asin((command.position - self.pad_width)/(2*self.finger_length))
        # publish message
        if self.invert:
            self.pub.publish(-angle + self.center)
        else:
            self.pub.publish(angle + self.center)

    def getPosition(self, joint_states):
        # TODO
        return 0.0

    def getEffort(self, joint_states):
        # TODO
        return 1.0


class GripperActionController:
    """ The actual action callbacks. """
    def __init__(self):
        rospy.init_node('gripper_controller')

        # setup model
        try:
            model = rospy.get_param('~model')
        except:
            rospy.logerr('no model specified, exiting')
            exit()
        if model == 'dualservo':
            self.model = TrapezoidGripperModel()
        elif model == 'parallel':
            self.model = ParallelGripperModel()
        elif model == 'singlesided':
            self.model = OneSideGripperModel()
        else:
            rospy.logerr('unknown model specified, exiting')
            exit()

        # subscribe to joint_states
        rospy.Subscriber('joint_states', JointState, self.stateCb)

        # subscribe to command and then spin
        self.server = actionlib.SimpleActionServer('~gripper_action', GripperCommandAction, execute_cb=self.actionCb, auto_start=False)
        self.server.start()
        rospy.spin()

    def actionCb(self, goal):
        """ Take an input command of width to open gripper. """
        rospy.loginfo('Gripper controller action goal recieved:%f' % goal.command.position)
        # send command to gripper
        self.model.setCommand(goal.command)
        # publish feedback
        while True:
            if self.server.is_preempt_requested():
                self.server.set_preemtped()
                rospy.loginfo('Gripper Controller: Preempted.')
                return
            # TODO: get joint position, break when we have reached goal
            break
        self.server.set_succeeded()
        rospy.loginfo('Gripper Controller: Succeeded.')

    def stateCb(self, msg):
        self.state = msg

if __name__=='__main__':
    try:
        rospy.logwarn("Please use gripper_controller (no .py)")
        GripperActionController()
    except rospy.ROSInterruptException:
        rospy.loginfo('Hasta la Vista...')

