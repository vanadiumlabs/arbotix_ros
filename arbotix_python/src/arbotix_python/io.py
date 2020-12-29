#!/usr/bin/env python3

"""
  io.py - ROS wrappers for ArbotiX I/O
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

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
from arbotix_msgs.msg import *

class DigitalServo:
    """ Class for a digital output. """
    def __init__(self, name, pin, value, rate, device):
        self.device = device
        self.value = value
        self.direction = 0
        self.pin = pin
        self.device.setDigital(self.pin, self.value, self.direction)
        rospy.Subscriber('~'+name, Digital, self.stateCb)
        self.t_delta = rospy.Duration(1.0/rate)
        self.t_next = rospy.Time.now() + self.t_delta
    def stateCb(self, msg):
        self.value = msg.value
        self.direction = msg.direction
    def update(self):
        if rospy.Time.now() > self.t_next:
            self.device.setDigital(self.pin, self.value, self.direction)
            self.t_next = rospy.Time.now() + self.t_delta

class DigitalSensor:
    """ Class for a digital input. """
    def __init__(self, name, pin, value, rate, device):
        self.device = device
        self.pin = pin
        self.device.setDigital(pin, value, 0)
        self.pub = rospy.Publisher('~'+name, Digital, queue_size=5)
        self.t_delta = rospy.Duration(1.0/rate)
        self.t_next = rospy.Time.now() + self.t_delta
    def update(self):
        if rospy.Time.now() > self.t_next:
            msg = Digital()
            msg.header.stamp = rospy.Time.now()
            msg.value = self.device.getDigital(self.pin)
            self.pub.publish(msg)
            self.t_next = rospy.Time.now() + self.t_delta

class AnalogSensor:
    """ Class for an analog input. """
    def __init__(self, name, pin, value, rate, leng, device):
        self.device = device
        self.pin = pin
        self.device.setDigital(pin, value, 0)
        self.pub = rospy.Publisher('~'+name, Analog, queue_size=5)
        self.t_delta = rospy.Duration(1.0/rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.leng = leng
    def update(self):
        if rospy.Time.now() > self.t_next:
            msg = Analog()
            msg.header.stamp = rospy.Time.now()
            msg.value = self.device.getAnalog(self.pin, self.leng)
            if msg.value >= 0:
                self.pub.publish(msg)
            self.t_next = rospy.Time.now() + self.t_delta

