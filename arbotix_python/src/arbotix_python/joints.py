#!/usr/bin/env python

# Copyright (c) 2010-2011 Vanadium Labs LLC. 
# All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of Vanadium Labs LLC nor the names of its 
#     contributors may be used to endorse or promote products derived 
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## @file joints.py Base class and support functions for joints.

## @brief Joints hold current values.
class Joint:

    ## @brief Constructs a Joint instance.
    ##
    ## @param device The arbotix instance.
    ## 
    ## @param name The joint name.
    def __init__(self, device, name):
        self.device = device
        self.name = name
        self.controller = None

        self.position = 0.0
        self.velocity = 0.0
        self.last = rospy.Time.now()

    ## @brief Get new output, in raw data format.
    ##
    ## @param frame The frame length in seconds to interpolate forward.
    ##
    ## @return The new output, in raw data format. 
    def interpolate(self, frame):
        return None

    ## @brief Set the current position from feedback data.
    ##
    ## @param raw_data The current feedback.
    ##
    ## @return The current position, in radians/meters. 
    def setCurrentFeedback(self, raw_data):
        return None

    ## @brief Set the goal position.
    ##
    ## @param position The goal position, in radians/meters. 
    ##
    ## @return The output position, in raw data format. 
    def setControlOutput(self, position):
        return None

    ## @brief Get a diagnostics message for this joint.
    ##
    ## @return Diagnostics message. 
    def getDiagnostics(self):
        return None


import rospy
import xml.dom.minidom

from math import pi, radians

## @brief Get joint parameters from URDF
def getJointsFromURDF():
    try:
        description = rospy.get_param("robot_description")
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        joints = {}
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                  continue
                name = child.getAttribute('name')
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))

                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
                joints[name] = joint
        return joints
    except:
        rospy.loginfo('No URDF defined, proceeding with defaults')
        return dict()


## @brief Get limits of servo, from YAML, then URDF, then defaults if neither is defined.
def getJointLimits(name, joint_defaults, default_min=-150, default_max=150):
    min_angle = radians(default_min)
    max_angle = radians(default_max)
    
    try: 
        min_angle = joint_defaults[name]['min']
    except:
        pass
    try: 
        min_angle = radians(rospy.get_param("/arbotix/dynamixels/"+name+"/min_angle"))
    except:
        pass
    try: 
        min_angle = radians(rospy.get_param("/arbotix/joints/"+name+"/min_angle"))
    except:
        pass
    try: 
        min_angle = rospy.get_param("/arbotix/joints/"+name+"/min_position")
    except:
        pass

    try: 
        max_angle = joint_defaults[name]['max']
    except:
        pass
    try: 
        max_angle = radians(rospy.get_param("/arbotix/dynamixels/"+name+"/max_angle"))
    except:
        pass
    try: 
        max_angle = radians(rospy.get_param("/arbotix/joints/"+name+"/max_angle"))
    except:
        pass
    try: 
        max_angle = rospy.get_param("/arbotix/joints/"+name+"/max_position")
    except:
        pass

    return (min_angle, max_angle)

