/* 
  Common Definitions for ROS driver ArbotiX Firmware
  Copyright (c) 2008-2012 Vanadium Labs LLC.  All right reserved.
 
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
*/ 

/* ArbotiX (id:253) Instruction Definitions */
#define ARB_SIZE_POSE       7    // pose size: a single param for size of pose
#define ARB_LOAD_POSE       8    // load pose: index, then pose positions (# of params = 2*pose_size)
#define ARB_LOAD_SEQ        9    // seq size: a single param for the size of the seq
#define ARB_PLAY_SEQ        10   // load seq: index/times (# of params = 3*seq_size)
#define ARB_LOOP_SEQ        11   // play seq: no params
//#define ARB_TEST          25   // hardware test: no params
#define ARB_CONTROL_SETUP   26   // write ids: id of controller, params (usually ids of servos, # of params = pose_size + 1)
#define ARB_CONTROL_WRITE   27   // write positions: positions in order of servos (# of params = 2*pose_size)
#define ARB_CONTROL_STAT    28   // retrieve status: id of controller
#define ARB_SYNC_READ       0x84

/* ArbotiX (id:253) Register Table Definitions */
#define REG_MODEL_NUMBER_L  0
#define REG_MODEL_NUMBER_H  1
#define REG_VERSION         2
#define REG_ID              3
#define REG_BAUD_RATE       4

#define REG_DIGITAL_IN0     5  // First block of digital pins to read
#define REG_DIGITAL_IN1     6
#define REG_DIGITAL_IN2     7
#define REG_DIGITAL_IN3     8

#define REG_RESCAN          15
#define REG_RETURN_LEVEL    16
#define REG_ALARM_LED       17
#define REG_ANA_BASE        18  // First Analog Port
#define REG_SERVO_BASE      26  // Up to 10 servos, each uses 2 bytes (L, then H), pulse width (0, 1000-2000ms)
#define REG_MOVING          46

#define REG_DIGITAL_OUT0    47  // First digital pin to write
                                // base + index, bit 1 = value (0,1), bit 0 = direction (0,1)

#define REG_RESERVED        79  // 79 -- 99 are reserved for future use
#define REG_USER            100 // 

/* Packet Decoding */
int mode = 0;                   // where we are in the frame

unsigned char id = 0;           // id of this frame
unsigned char length = 0;       // length of this frame
unsigned char ins = 0;          // instruction of this frame

unsigned char params[143];      // parameters (match RX-64 buffer size)
unsigned char index = 0;        // index in param buffer

int checksum;                   // checksum
