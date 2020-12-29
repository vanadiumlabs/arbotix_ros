#!/usr/bin/env python3

"""
  diagnostics.py - diagnostic output code
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
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState

class DiagnosticsPublisher:
    """ Class to handle publications of joint_states message. """

    def __init__(self):
        self.t_delta = rospy.Duration(1.0/rospy.get_param("~diagnostic_rate", 1.0))
        self.t_next = rospy.Time.now() + self.t_delta
        self.pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=5)

    def update(self, joints, controllers):
        """ Publish diagnostics. """    
        now = rospy.Time.now()
        if now > self.t_next:
            # create message
            msg = DiagnosticArray()
            msg.header.stamp = now
            for controller in controllers:
                d = controller.getDiagnostics()
                if d:
                    msg.status.append(d)
            for joint in joints:
                d = joint.getDiagnostics()
                if d:
                    msg.status.append(d)
            # publish and update stats
            self.pub.publish(msg)
            self.t_next = now + self.t_delta
        

class JointStatePublisher:
    """ Class to handle publications of joint_states message. """

    def __init__(self):
        # parameters: throttle rate and geometry
        self.rate = rospy.get_param("~read_rate", 10.0)
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

        # subscriber
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)

    def update(self, joints, controllers):
        """ publish joint states. """
        if rospy.Time.now() > self.t_next:   
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = list()
            msg.position = list()
            msg.velocity = list()
            for joint in joints:
                msg.name.append(joint.name)
                msg.position.append(joint.position)
                msg.velocity.append(joint.velocity)
            for controller in controllers:
                msg.name += controller.joint_names
                msg.position += controller.joint_positions
                msg.velocity += controller.joint_velocities
            self.pub.publish(msg)
            self.t_next = rospy.Time.now() + self.t_delta

