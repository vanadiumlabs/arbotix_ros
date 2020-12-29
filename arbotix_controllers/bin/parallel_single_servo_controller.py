#!/usr/bin/env python3

"""
  parallel_single_servo_controller.py - controls a single-servo parallel-jaw gripper
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

import rospy, tf
import thread

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from math import asin

class ParallelGripperController:
    """ A simple controller that operates a single servo parallel jaw gripper. """
    def __init__(self):
        rospy.init_node("gripper_controller")

        # TODO: load calibration data. Form: opening->servo angle
        self.calib = { 0.0000 : 1.8097, 0.0159: 1.2167, 0.0254 : 0.8997, 0.0381 : 0.4499, 0.042 : 0.1943 }
        #self.calib = { 0.0000 : 866, 0.0159: 750, 0.0254 : 688, 0.0381 : 600, 0.042 : 550 }

        # parameters
        self.min = rospy.get_param("~min", 0.0)
        self.max = rospy.get_param("~max", 0.042)
        self.center = rospy.get_param("~center", 512)
        self.invert = rospy.get_param("~invert", False)
        
        # publishers
        self.commandPub = rospy.Publisher("gripper_joint/command", Float64, queue_size=5)
        self.br = tf.TransformBroadcaster()
    
        # current width of gripper
        self.width = 0.0

        # subscribe to command and then spin
        rospy.Subscriber("~command", Float64, self.commandCb)
        rospy.Subscriber("joint_states", JointState, self.stateCb)
        
        r = rospy.Rate(15)
        while not rospy.is_shutdown():
            # output tf
            self.br.sendTransform((0, -self.width/2.0, 0),
                                   tf.transformations.quaternion_from_euler(0, 0, 0),
                                   rospy.Time.now(),
                                   "gripper_left_link",
                                   "gripper_link")
            self.br.sendTransform((0, self.width/2.0, 0),
                                   tf.transformations.quaternion_from_euler(0, 0, 0),
                                   rospy.Time.now(),
                                   "gripper_right_link",
                                   "gripper_link")
            r.sleep()

    def getCommand(self, width):
        """ Get servo command for an opening width. """
        keys = self.calib.keys(); keys.sort()   
        # find end points of segment
        low = keys[0]; 
        high = keys[-1]
        for w in keys[1:-1]:
            if w > low and w < width:
                low = w
            if w < high and w > width:
                high = w
        # linear interpolation
        scale = (width-low)/(high-low)
        return ((self.calib[high]-self.calib[low])*scale) + self.calib[low]

    def getWidth(self, command):
        """ Get opening width for a particular servo command. """
        reverse_calib = dict()
        for k, v in self.calib.items():
            reverse_calib[v] = k
        keys = reverse_calib.keys(); keys.sort()
        # find end points of segment
        low = keys[0]
        high = keys[-1]
        for c in keys[1:-1]:
            if c > low and c < command:
                low = c
            if c < high and c > command:
                high = c
        # linear interpolation
        scale = (command-low)/(high-low)
        return ((reverse_calib[high]-reverse_calib[low])*scale) + reverse_calib[low]

    def commandCb(self, msg):
        """ Take an input command of width to open gripper. """
        # check limits
        if msg.data > self.max or msg.data < self.min:
            rospy.logerr("Command exceeds limits.")
            return
        # compute angle
        self.commandPub.publish( Float64( self.getCommand(msg.data) ) )

    def stateCb(self, msg):
        """ The callback that listens for joint_states. """
        try:
            index = msg.name.index("gripper_joint")
        except ValueError:
            return
        self.width = self.getWidth(msg.position[index])

if __name__=="__main__": 
    try:
        ParallelGripperController()
    except rospy.ROSInterruptException:
        rospy.loginfo("Hasta la Vista...")
        
