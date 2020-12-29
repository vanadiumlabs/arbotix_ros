#!/usr/bin/env python3

"""
  sensors.py - various conversions from raw analog to sensor range
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

from sensor_msgs.msg import Range

class SharpIR:
    radiation_type = Range.INFRARED
    field_of_view = 0.001
    min_range = 0.0
    max_range = 1.0

    def convert(self, raw):
        """ Convert raw analog (10-bit) to distance. """
        return raw

class gpA710YK(SharpIR):
    """ Ultra long-range Sharp IR sensor. """
    min_range = 0.75
    max_range = 5.50

    def convert(self, raw):
        """ Convert raw analog (10-bit) to distance. """
        if raw > 100:
            return (497.0/(raw-56))
        else:
            return self.max_range+0.1
        
class gpA02YK(SharpIR):
    min_range = 0.20
    max_range = 1.50

    def convert(self, raw):
        """ Convert raw analog (10-bit) to distance. """
        if raw > 80:
            return (115.0/(raw-12))
        else:
            return self.max_range+0.1

class gp2d12(SharpIR):
    """ The typical GP2D12 IR ranger. """
    min_range = 0.10
    max_range = 0.80

    def convert(self, raw):
        """ Convert raw analog (10-bit) to distance. """
        if raw > 40:
            return (52.0/(raw-12))
        else:
            return self.max_range+0.1

class maxSonar():
    radiation_type = Range.ULTRASOUND
    field_of_view = 0.785398163
    min_range = 0.0
    max_range = 6.4516 

    def convert(self, raw):
        """ Convert raw analog (10-bit) to distance. """
        return 12.7 * (raw/1024.0)

