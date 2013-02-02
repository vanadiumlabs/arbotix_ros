#!/usr/bin/env python

"""
  linear_controller.py - controller for a linear actuator with analog feedback
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

import rospy, actionlib

from joints import *
from controllers import *
from std_msgs.msg import Float64
from diagnostic_msgs.msg import *
from std_srvs.srv import *

from struct import unpack

class LinearJoint(Joint):
    def __init__(self, device, name):
        Joint.__init__(self, device, name)

        self.dirty = False
        self.position = 0.0                     # current position, as returned by feedback (meters)
        self.desired = 0.0                      # desired position (meters)
        self.velocity = 0.0                     # moving speed
        self.last = rospy.Time.now()

        # TODO: load these from URDF
        self.min = rospy.get_param('~joints/'+name+'/min_position',0.0)
        self.max = rospy.get_param('~joints/'+name+'/max_position',0.5)
        self.max_speed = rospy.get_param('~joints/'+name+'/max_speed',0.0508)

        # calibration data {reading: position}
        self.cal = { -1: -1, 1: 1 }
        self.cal_raw = rospy.get_param('~joints/'+name+'/calibration_data', self.cal)
        self.cal = dict()
        for key, value in self.cal_raw.items():
            self.cal[int(key)] = value
        self.keys = sorted(self.cal.keys())

        rospy.Subscriber(name+'/command', Float64, self.commandCb)
        
    def interpolate(self, frame):
        """ Get new output: 1 = increase position, -1 is decrease position. """
        if self.dirty:
            cmd = self.desired - self.position
            if self.device.fake: 
                self.position = self.desired
                self.dirty = False
                return None
            if cmd > 0.01:
                return 1
            elif cmd < -0.01:
                return -1
            else:
                self.dirty = False
                return 0
        else:
            return None

    def setCurrentFeedback(self, reading):
        if reading >= self.keys[0] and reading <= self.keys[-1]:
            last_angle = self.position
            self.position = self.readingToPosition(reading)
            # update velocity estimate
            t = rospy.Time.now()
            self.velocity = (self.position - last_angle)/((t - self.last).to_nsec()/1000000000.0)
            self.last = t
        else:
            rospy.logerr(self.name + ": feedback reading out of range")

    def setControlOutput(self, position):
        """ Set the position that controller is moving to. 
            Returns output value in raw_data format. """
        if position <= self.max and position >= self.min:
            self.desired = position
            self.dirty = True
        else:
            rospy.logerr(self.name + ": requested position is out of range: " + str(position))
        return None # TODO
    
    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        if self.dirty:
            msg.message = "Moving"
        else:
            msg.message = "OK"
        msg.values.append(KeyValue("Position", str(self.position)))
        return msg

    def commandCb(self, req):
        """ Float64 style command input. """
        if self.device.fake:
            self.position = req.data
        else:
            if req.data <= self.max and req.data >= self.min:
                self.desired = req.data
                self.dirty = True
            else:
                rospy.logerr(self.name + ": requested position is out of range: " + str(req))

    def readingToPosition(self, reading):
        low = 0
        while reading > self.keys[low+1]:
            low += 1
        high = len(self.keys) - 1
        while reading < self.keys[high-1]:
            high += -1
        x = self.keys[high] - self.keys[low]
        y = self.cal[self.keys[high]] - self.cal[self.keys[low]]
        x1 = reading - self.keys[low]
        y1 = y * ( float(x1)/float(x) )
        return self.cal[self.keys[low]] + y1


class LinearControllerAbsolute(Controller):
    """ A controller for a linear actuator, with absolute positional feedback. """

    def __init__(self, device, name):
        Controller.__init__(self, device, name)

        self.a = rospy.get_param('~controllers/'+name+'/motor_a',29)
        self.b = rospy.get_param('~controllers/'+name+'/motor_b',30)
        self.p = rospy.get_param('~controllers/'+name+'/motor_pwm',31)
        self.analog = rospy.get_param('~controllers/'+name+'/feedback',0)
        self.last = 0
        self.last_reading = 0

        self.delta = rospy.Duration(1.0/rospy.get_param('~controllers/'+name+'/rate', 10.0))
        self.next = rospy.Time.now() + self.delta

        self.joint = device.joints[rospy.get_param('~controllers/'+name+'/joint')]

        rospy.loginfo("Started LinearController ("+self.name+").")

    def startup(self):
        if not self.fake:
            self.joint.setCurrentFeedback(self.device.getAnalog(self.analog))

    def update(self):
        now = rospy.Time.now()
        if now > self.next:
            # read current position
            if self.joint.dirty:
                if not self.fake:
                    try:
                        self.last_reading = self.getPosition()
                        self.joint.setCurrentFeedback(self.last_reading)
                    except Exception as e:
                        print "linear error: ", e
                # update movement
                output = self.joint.interpolate(1.0/self.delta.to_sec())
                if self.last != output and not self.fake: 
                    self.setSpeed(output)
                    self.last = output
            self.next = now + self.delta
    
    def setSpeed(self, speed):
        """ Set speed of actuator. """
        if speed > 0:
            self.device.setDigital(self.a, 0); self.device.setDigital(self.b, 1);   # up
            self.device.setDigital(self.p, 1)
        elif speed < 0:
            self.device.setDigital(self.a, 1); self.device.setDigital(self.b, 0);   # down
            self.device.setDigital(self.p, 1)
        else:
            self.device.setDigital(self.p, 0)

    def getPosition(self):
        return self.device.getAnalog(self.analog)

    def shutdown(self):
        if not self.fake:
            self.device.setDigital(self.p, 0)

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name

        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        if not self.fake:
            msg.values.append(KeyValue("Encoder Reading", str(self.last_reading)))
        
        return msg


class LinearControllerIncremental(LinearControllerAbsolute):
    """ A controller for a linear actuator, without absolute encoder. """
    POSITION_L  = 100
    POSITION_H  = 101
    DIRECTION   = 102

    def __init__(self, device, name):
        Controller.__init__(self, device, name)
        self.pause = True

        self.a = rospy.get_param('~controllers/'+name+'/motor_a',29)
        self.b = rospy.get_param('~controllers/'+name+'/motor_b',30)
        self.p = rospy.get_param('~controllers/'+name+'/motor_pwm',31)
        self.last = 0
        self.last_reading = 0

        self.delta = rospy.Duration(1.0/rospy.get_param('~controllers/'+name+'/rate', 10.0))
        self.next = rospy.Time.now() + self.delta

        self.joint = device.joints[rospy.get_param('~controllers/'+name+'/joint')]

        rospy.Service(name+'/zero', Empty, self.zeroCb)
        rospy.loginfo("Started LinearControllerIncremental ("+self.name+").")

    def startup(self):
        if not self.fake:
            self.zeroEncoder()

    def setSpeed(self, speed):
        """ Set speed of actuator. We need to set direction for encoder. """
        if speed > 0:
            self.device.write(253, self.DIRECTION, [1])
            self.device.setDigital(self.a, 0); self.device.setDigital(self.b, 1);   # up
            self.device.setDigital(self.p, 1)
        elif speed < 0:
            self.device.write(253, self.DIRECTION, [0])
            self.device.setDigital(self.a, 1); self.device.setDigital(self.b, 0);   # down
            self.device.setDigital(self.p, 1)
        else:
            self.device.setDigital(self.p, 0)

    def getPosition(self):
        return unpack('=h', "".join([chr(k) for k in self.device.read(253, self.POSITION_L, 2)]) )[0]

    def zeroEncoder(self, timeout=15.0):
        rospy.loginfo(self.name + ': zeroing encoder')
        self.setSpeed(1)
        last_pos = None
        for i in range(int(timeout)):
            if rospy.is_shutdown():
                return
            try:
                new_pos = self.getPosition()
            except:
                pass
            if last_pos == new_pos:
                break
            last_pos = new_pos
            rospy.sleep(1)
        self.setSpeed(0)
        self.device.write(253, self.POSITION_L, [0, 0])
        self.joint.setCurrentFeedback(0)

    def zeroCb(self, msg):
        if not self.fake:
            self.zeroEncoder(15.0)
        return EmptyResponse()

    def shutdown(self):
        if not self.fake:
            self.setSpeed(0)

