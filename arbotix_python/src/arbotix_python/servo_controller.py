#!/usr/bin/env python

"""
  servo_controller.py: classes for servo interaction
  Copyright (c) 2011-2013 Vanadium Labs LLC. All right reserved.

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

from math import radians

from std_msgs.msg import Float64
from arbotix_msgs.srv import *
from diagnostic_msgs.msg import *

from ax12 import *
from joints import *

class DynamixelServo(Joint):

    def __init__(self, device, name, ns="~joints"):
        Joint.__init__(self, device, name)
        n = ns+"/"+name+"/"
        
        self.id = int(rospy.get_param(n+"id"))
        self.ticks = rospy.get_param(n+"ticks", 1024)
        self.neutral = rospy.get_param(n+"neutral", self.ticks/2)
        if self.ticks == 4096:
            self.range = 360.0
        else:
            self.range = 300.0
        self.range = rospy.get_param(n+"range", self.range)
        self.rad_per_tick = radians(self.range)/self.ticks

        # TODO: load these from URDF
        self.max_angle = radians(rospy.get_param(n+"max_angle",self.range/2.0))
        self.min_angle = radians(rospy.get_param(n+"min_angle",-self.range/2.0))
        self.max_speed = radians(rospy.get_param(n+"max_speed",684.0)) 
                                                # max speed = 114 rpm - 684 deg/s
        self.invert = rospy.get_param(n+"invert",False)
        self.readable = rospy.get_param(n+"readable",True)

        self.status = "OK"
        self.level = DiagnosticStatus.OK

        self.dirty = False                      # newly updated position?
        self.position = 0.0                     # current position, as returned by servo (radians)
        self.desired = 0.0                      # desired position (radians)
        self.last_cmd = 0.0                     # last position sent (radians)
        self.velocity = 0.0                     # moving speed
        self.enabled = True                     # can we take commands?
        self.active = False                     # are we under torque control?
        self.last = rospy.Time.now()

        self.reads = 0.0                        # number of reads
        self.errors = 0                         # number of failed reads
        self.total_reads = 0.0                  
        self.total_errors = [0.0]

        self.voltage = 0.0
        self.temperature = 0.0
        
        # ROS interfaces
        rospy.Subscriber(name+'/command', Float64, self.commandCb)
        rospy.Service(name+'/relax', Relax, self.relaxCb)
        rospy.Service(name+'/enable', Enable, self.enableCb)
        rospy.Service(name+'/set_speed', SetSpeed, self.setSpeedCb)

    def interpolate(self, frame):
        """ Get the new position to move to, in ticks. """
        if self.enabled and self.active and self.dirty:
            # compute command, limit velocity
            cmd = self.desired - self.last_cmd
            if cmd > self.max_speed/frame:
                cmd = self.max_speed/frame
            elif cmd < -self.max_speed/frame:
                cmd = -self.max_speed/frame
            # compute angle, apply limits
            ticks = self.angleToTicks(self.last_cmd + cmd)
            self.last_cmd = self.ticksToAngle(ticks)
            self.speed = cmd*frame
            # cap movement
            if self.last_cmd == self.desired:
                self.dirty = False
            # when fake, need to set position/velocity here
            if self.device.fake:
                last_angle = self.position
                self.position = self.last_cmd
                t = rospy.Time.now()
                self.velocity = (self.position - last_angle)/((t - self.last).to_nsec()/1000000000.0)
                self.last = t
                return None
            return int(ticks)
        else:
            # when fake, need to reset velocity to 0 here.
            if self.device.fake:
                self.velocity = 0.0
                self.last = rospy.Time.now()
            return None

    def setCurrentFeedback(self, reading):
        """ Update angle in radians by reading from servo, or by 
            using position passed in from a sync read (in ticks). """
        if reading > -1 and reading < self.ticks:     # check validity
            self.reads += 1
            self.total_reads += 1
            last_angle = self.position
            self.position = self.ticksToAngle(reading)
            # update velocity estimate
            t = rospy.Time.now()
            self.velocity = (self.position - last_angle)/((t - self.last).to_nsec()/1000000000.0)
            self.last = t
        else:
            rospy.logdebug("Invalid read of servo: id " + str(self.id) + ", value " + str(reading))
            self.errors += 1
            self.total_reads += 1
            return
        if not self.active:
            self.last_cmd = self.position

    def setControlOutput(self, position):
        """ Set the position that controller is moving to. 
            Returns output value in ticks. """
        if self.enabled:
            ticks = self.angleToTicks(position)
            self.desired = position
            self.dirty = True
            self.active = True
            return int(ticks)
        return -1

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        if self.temperature > 60:   # TODO: read this value from eeprom
            self.status = "OVERHEATED, SHUTDOWN"
            self.level = DiagnosticStatus.ERROR
        elif self.temperature > 50 and self.status != "OVERHEATED, SHUTDOWN":
            self.status = "OVERHEATING"
            self.level = DiagnosticStatus.WARN
        elif self.status != "OVERHEATED, SHUTDOWN":
            self.status = "OK"
            self.level = DiagnosticStatus.OK
        msg.level = self.level
        msg.message = self.status
        msg.values.append(KeyValue("Position", str(self.position)))
        msg.values.append(KeyValue("Temperature", str(self.temperature)))
        msg.values.append(KeyValue("Voltage", str(self.voltage)))
        if self.reads + self.errors > 100:
            self.total_errors.append((self.errors*100.0)/(self.reads+self.errors))
            if len(self.total_errors) > 10:
                self.total_errors = self.total_errors[-10:]
            self.reads = 0
            self.errors = 0
        msg.values.append(KeyValue("Reads", str(self.total_reads)))
        msg.values.append(KeyValue("Error Rate", str(sum(self.total_errors)/len(self.total_errors))+"%" ))
        if self.active:
            msg.values.append(KeyValue("Torque", "ON"))
        else:
            msg.values.append(KeyValue("Torque", "OFF"))
        return msg

    def angleToTicks(self, angle):
        """ Convert an angle to ticks, applying limits. """
        ticks = self.neutral + (angle/self.rad_per_tick)
        if self.invert:
            ticks = self.neutral - (angle/self.rad_per_tick)
        if ticks >= self.ticks:
            return self.ticks-1.0
        if ticks < 0:
            return 0
        return ticks

    def ticksToAngle(self, ticks):
        """ Convert an ticks to angle, applying limits. """
        angle = (ticks - self.neutral) * self.rad_per_tick
        if self.invert:
            angle = -1.0 * angle
        return angle

    def speedToTicks(self, rads_per_sec):
        """ Convert speed in radians per second to ticks, applying limits. """
        ticks = self.ticks * rads_per_sec / self.max_speed
        if ticks >= self.ticks:
            return self.ticks-1.0
        if ticks < 0:
            return 0
        return ticks  

    def enableCb(self, req):
        """ Turn on/off servo torque, so that it is pose-able. """
        if req.enable:
            self.enabled = True
        else:
            if not self.device.fake:
                self.device.disableTorque(self.id)
            self.dirty = False
            self.enabled = False
            self.active = False
        return EnableResponse(self.enabled)

    def relaxCb(self, req):
        """ Turn off servo torque, so that it is pose-able. """
        if not self.device.fake:
            self.device.disableTorque(self.id)
        self.dirty = False
        self.active = False
        return RelaxResponse()

    def commandCb(self, req):
        """ Float64 style command input. """
        if self.enabled:
            if self.controller and self.controller.active():
                # Under and action control, do not interfere
                return
            elif self.desired != req.data or not self.active:
                self.dirty = True
                self.active = True
                self.desired = req.data
                
    def setSpeedCb(self, req):
        """ Set servo speed. Requested speed is in radians per second.
            Don't allow 0 which means "max speed" to a Dynamixel in joint mode. """
        if not self.device.fake:
            ticks_per_sec = max(1, int(self.speedToTicks(req.speed)))
            self.device.setSpeed(self.id, ticks_per_sec)
        return SetSpeedResponse()

class HobbyServo(Joint):

    def __init__(self, device, name, ns="~joints"):
        Joint.__init__(self, device, name)
        n = ns+"/"+name+"/"
        
        self.id = int(rospy.get_param(n+"id"))
        self.ticks = rospy.get_param(n+"ticks", 2000)
        self.neutral = rospy.get_param(n+"neutral", 1500)
        self.range = rospy.get_param(n+"range", 180)
        self.rad_per_tick = radians(self.range)/self.ticks

        # TODO: load these from URDF
        self.max_angle = radians(rospy.get_param(n+"max_angle",self.range/2.0))
        self.min_angle = radians(rospy.get_param(n+"min_angle",-self.range/2.0))
        self.max_speed = radians(rospy.get_param(n+"max_speed",90.0)) 

        self.invert = rospy.get_param(n+"invert",False)

        self.dirty = False                      # newly updated position?
        self.position = 0.0                     # current position, as returned by servo (radians)
        self.desired = 0.0                      # desired position (radians)
        self.last_cmd = 0.0                     # last position sent (radians)
        self.velocity = 0.0                     # moving speed
        self.last = rospy.Time.now()
        
        # ROS interfaces
        rospy.Subscriber(name+'/command', Float64, self.commandCb)

    def interpolate(self, frame):
        """ Get the new position to move to, in ticks. """
        if self.dirty:
            # compute command, limit velocity
            cmd = self.desired - self.last_cmd
            if cmd > self.max_speed/frame:
                cmd = self.max_speed/frame
            elif cmd < -self.max_speed/frame:
                cmd = -self.max_speed/frame
            # compute angle, apply limits
            ticks = self.angleToTicks(self.last_cmd + cmd)
            self.last_cmd = self.ticksToAngle(ticks)
            self.speed = cmd*frame
            # cap movement
            if self.last_cmd == self.desired:
                self.dirty = False
            if self.device.fake:
                self.position = self.last_cmd
                return None
            return int(ticks)
        else:
            return None

    def setCurrentFeedback(self, raw_data):
        """ Update angle in radians by reading from servo, or by 
            using position passed in from a sync read (in ticks). """
        return None

    def setControlOutput(self, position):
        """ Set the position that controller is moving to. 
            Returns output value in ticks. """
        ticks = self.angleToTicks(position)
        self.desired = position
        self.dirty = True
        return int(ticks)

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        msg.values.append(KeyValue("Position", str(self.position)))
        return msg

    def angleToTicks(self, angle):
        """ Convert an angle to ticks, applying limits. """
        ticks = self.neutral + (angle/self.rad_per_tick)
        if self.invert:
            ticks = self.neutral - (angle/self.rad_per_tick)
        if ticks >= self.ticks:
            return self.ticks-1.0
        if ticks < 0:
            return 0
        return ticks

    def ticksToAngle(self, ticks):
        """ Convert an ticks to angle, applying limits. """
        angle = (ticks - self.neutral) * self.rad_per_tick
        if self.invert:
            angle = -1.0 * angle
        return angle        

    def commandCb(self, req):
        """ Float64 style command input. """
        if self.controller and self.controller.active():
            # Under and action control, do not interfere
            return
        else:
            self.dirty = True
            self.desired = req.data


from controllers import *

class ServoController(Controller):

    def __init__(self, device, name):
        Controller.__init__(self, device, name)
        self.dynamixels = list()
        self.hobbyservos = list()
        self.iter = 0

        # steal some servos
        for joint in device.joints.values():
            if isinstance(joint, DynamixelServo):
                self.dynamixels.append(joint)
            elif isinstance(joint, HobbyServo):
                self.hobbyservos.append(joint)

        self.w_delta = rospy.Duration(1.0/rospy.get_param("~write_rate", 10.0))
        self.w_next = rospy.Time.now() + self.w_delta

        self.r_delta = rospy.Duration(1.0/rospy.get_param("~read_rate", 10.0))
        self.r_next = rospy.Time.now() + self.r_delta

        rospy.Service(name + '/relax_all', Relax, self.relaxCb)
        rospy.Service(name + '/enable_all', Enable, self.enableCb)

    def update(self):
        """ Read servo positions, update them. """
        if rospy.Time.now() > self.r_next and not self.fake:
            if self.device.use_sync_read:
                # arbotix/servostik/wifi board sync_read
                synclist = list()
                for joint in self.dynamixels:
                    if joint.readable:
                        synclist.append(joint.id)
                if len(synclist) > 0:
                    val = self.device.syncRead(synclist, P_PRESENT_POSITION_L, 2)
                    if val: 
                        for joint in self.dynamixels:
                            try:
                                i = synclist.index(joint.id)*2
                                joint.setCurrentFeedback(val[i]+(val[i+1]<<8))
                            except:
                                # not a readable servo
                                continue 
            else:
                # direct connection, or other hardware with no sync_read capability
                for joint in self.dynamixels:
                    joint.setCurrentFeedback(self.device.getPosition(joint.id))
            self.r_next = rospy.Time.now() + self.r_delta

        if rospy.Time.now() > self.w_next:
            if self.device.use_sync_write and not self.fake:
                syncpkt = list()
                for joint in self.dynamixels:
                    v = joint.interpolate(1.0/self.w_delta.to_sec())
                    if v != None:   # if was dirty
                        syncpkt.append([joint.id,int(v)%256,int(v)>>8])                         
                if len(syncpkt) > 0:      
                    self.device.syncWrite(P_GOAL_POSITION_L,syncpkt)
            else:
                for joint in self.dynamixels:
                    v = joint.interpolate(1.0/self.w_delta.to_sec())
                    if v != None:   # if was dirty      
                        self.device.setPosition(joint.id, int(v))
            for joint in self.hobbyservos: 
                v = joint.interpolate(1.0/self.w_delta.to_sec())
                if v != None:   # if it was dirty   
                    self.device.setServo(joint.id, v)
            self.w_next = rospy.Time.now() + self.w_delta

    def getDiagnostics(self):
        """ Update status of servos (voltages, temperatures). """
        if self.iter % 5 == 0 and not self.fake:
            if self.device.use_sync_read:
                # arbotix/servostik/wifi board sync_read
                synclist = list()
                for joint in self.dynamixels:
                    if joint.readable:
                        synclist.append(joint.id)
                if len(synclist) > 0:
                    val = self.device.syncRead(synclist, P_PRESENT_VOLTAGE, 2)
                    if val:
                        for joint in self.dynamixels:
                            try:
                                i = synclist.index(joint.id)*2
                                if val[i] < 250:
                                    joint.voltage = val[i]/10.0
                                if val[i+1] < 100:
                                    joint.temperature = val[i+1]
                            except:
                                # not a readable servo
                                continue 
            else:
                # direct connection, or other hardware with no sync_read capability
                for joint in self.dynamixels:
                    if joint.readable:
                        val = self.device.read(joint.id, P_PRESENT_VOLTAGE, 2)
                        try:
                            joint.voltage = val[0]
                            joint.temperature = val[1]
                        except:
                            continue
        self.iter += 1
        return None

    def enableCb(self, req):
        """ Turn on/off all servos torque, so that they are pose-able. """
        for joint in self.dynamixels:
            resp = joint.enableCb(req)
        return resp

    def relaxCb(self, req):
        """ Turn off all servos torque, so that they are pose-able. """
        for joint in self.dynamixels:
            resp = joint.relaxCb(req)
        return resp
