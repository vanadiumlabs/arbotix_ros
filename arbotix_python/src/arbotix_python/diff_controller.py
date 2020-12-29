#!/usr/bin/env python3

"""
  diff_controller.py - controller for a differential drive
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

from math import sin,cos,pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import *
from tf.broadcaster import TransformBroadcaster

from arbotix_python.ax12 import *
from arbotix_python.controllers import *
from struct import unpack

class DiffController(Controller):
    """ Controller to handle movement & odometry feedback for a differential 
            drive mobile base. """
    def __init__(self, device, name):
        Controller.__init__(self, device, name)
        self.pause = True
        self.last_cmd = rospy.Time.now()

        # parameters: rates and geometry
        self.rate = rospy.get_param('~controllers/'+name+'/rate',10.0)
        self.timeout = rospy.get_param('~controllers/'+name+'/timeout',1.0)
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.ticks_meter = float(rospy.get_param('~controllers/'+name+'/ticks_meter'))
        self.base_width = float(rospy.get_param('~controllers/'+name+'/base_width'))

        self.base_frame_id = rospy.get_param('~controllers/'+name+'/base_frame_id', 'base_link')
        self.odom_frame_id = rospy.get_param('~controllers/'+name+'/odom_frame_id', 'odom')

        # parameters: PID
        self.Kp = rospy.get_param('~controllers/'+name+'/Kp', 5)
        self.Kd = rospy.get_param('~controllers/'+name+'/Kd', 1)
        self.Ki = rospy.get_param('~controllers/'+name+'/Ki', 0)
        self.Ko = rospy.get_param('~controllers/'+name+'/Ko', 50)

        # parameters: acceleration
        self.accel_limit = rospy.get_param('~controllers/'+name+'/accel_limit', 0.1)
        self.max_accel = int(self.accel_limit*self.ticks_meter/self.rate)

        # output for joint states publisher
        self.joint_names = ["base_l_wheel_joint","base_r_wheel_joint"]
        self.joint_positions = [0,0]
        self.joint_velocities = [0,0]

        # internal data            
        self.v_left = 0                 # current setpoint velocity
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0
        self.dx = 0                     # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()    # time for determining dx/dy

        # subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()
		
        rospy.loginfo("Started DiffController ("+name+"). Geometry: " + str(self.base_width) + "m wide, " + str(self.ticks_meter) + " ticks/m.")

    def startup(self):
        if not self.fake:
            self.setup(self.Kp,self.Kd,self.Ki,self.Ko) 
    
    def update(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            if self.fake:
                x = cos(self.th)*self.dx*elapsed
                y = -sin(self.th)*self.dx*elapsed
                self.x += cos(self.th)*self.dx*elapsed
                self.y += sin(self.th)*self.dx*elapsed
                self.th += self.dr*elapsed
            else:
                # read encoders
                try:
                    left, right = self.status()
                except Exception as e:
                    rospy.logerr("Could not update encoders: " + str(e))
                    return
                rospy.logdebug("Encoders: " + str(left) +","+ str(right))

                # calculate odometry
                if self.enc_left == None:
                    d_left = 0
                    d_right = 0
                else:
                    d_left = (left - self.enc_left)/self.ticks_meter
                    d_right = (right - self.enc_right)/self.ticks_meter
                self.enc_left = left
                self.enc_right = right

                d = (d_left+d_right)/2
                th = (d_right-d_left)/self.base_width
                self.dx = d / elapsed
                self.dr = th / elapsed

                if (d != 0):
                    x = cos(th)*d
                    y = -sin(th)*d
                    self.x = self.x + (cos(self.th)*x - sin(self.th)*y)
                    self.y = self.y + (sin(self.th)*x + cos(self.th)*y)
                if (th != 0):
                    self.th = self.th + th

            # publish or perish
            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(self.th/2)
            quaternion.w = cos(self.th/2)
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0), 
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)

            if now > (self.last_cmd + rospy.Duration(self.timeout)):
                self.v_des_left = 0
                self.v_des_right = 0

            # update motors
            if not self.fake:
                if self.v_left < self.v_des_left:
                    self.v_left += self.max_accel
                    if self.v_left > self.v_des_left:
                        self.v_left = self.v_des_left
                else:
                    self.v_left -= self.max_accel
                    if self.v_left < self.v_des_left:
                        self.v_left = self.v_des_left
                
                if self.v_right < self.v_des_right:
                    self.v_right += self.max_accel
                    if self.v_right > self.v_des_right:
                        self.v_right = self.v_des_right
                else:
                    self.v_right -= self.max_accel
                    if self.v_right < self.v_des_right:
                        self.v_right = self.v_des_right
                self.write(self.v_left, self.v_right)

            self.t_next = now + self.t_delta
 
    def shutdown(self):
        if not self.fake:
            self.write(0,0)

    def cmdVelCb(self,req):
        """ Handle movement requests. """
        self.last_cmd = rospy.Time.now()
        if self.fake:
            self.dx = req.linear.x        # m/s
            self.dr = req.angular.z       # rad/s
        else:
            # set motor speeds in ticks per 1/30s
            self.v_des_left = int( ((req.linear.x - (req.angular.z * self.base_width/2.0)) * self.ticks_meter) / 30.0)
            self.v_des_right = int( ((req.linear.x + (req.angular.z * self.base_width/2.0)) * self.ticks_meter) / 30.0)

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        if not self.fake:
            msg.values.append(KeyValue("Left", str(self.enc_left)))
            msg.values.append(KeyValue("Right", str(self.enc_right)))
        msg.values.append(KeyValue("dX", str(self.dx)))
        msg.values.append(KeyValue("dR", str(self.dr)))
        return msg

    ###
    ### Controller Specification: 
    ###
    ###  setup: Kp, Kd, Ki, Ko (all unsigned char)
    ###
    ###  write: left_speed, right_speed (2-byte signed, ticks per frame)
    ###
    ###  status: left_enc, right_enc (4-byte signed)
    ### 
    
    def setup(self, kp, kd, ki, ko):
        success = self.device.execute(253, AX_CONTROL_SETUP, [10, kp, kd, ki, ko])

    def write(self, left, right):
        """ Send a closed-loop speed. Base PID loop runs at 30Hz, these values
                are therefore in ticks per 1/30 second. """
        left = left&0xffff
        right = right&0xffff
        success = self.device.execute(253, AX_CONTROL_WRITE, [10, left%256, left>>8, right%256, right>>8])

    def status(self):
        """ read 32-bit (signed) encoder values. """
        values = self.device.execute(253, AX_CONTROL_STAT, [10])
        left_values = "".join([chr(k) for k in values[0:4] ])        
        right_values = "".join([chr(k) for k in values[4:] ])
        try:
            left = unpack('=l',left_values)[0]
            right = unpack('=l',right_values)[0]
            return [left, right]
        except:
            return None

