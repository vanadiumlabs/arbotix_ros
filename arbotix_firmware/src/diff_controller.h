/* 
  ArbotiX Firmware for ROS driver
  Copyright (c) 2009-2011 Vanadium Labs LLC.  All right reserved.
 
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

#include <Arduino.h>
#include <math.h>

/* Register Storage */
int left_pwm;
int right_pwm;
int left_speed;
int right_speed;

/* PID Parameters */
int Kp;
int Kd;
int Ki;
int Ko;                      // output scale
int maxAccel;                // maximum acceleration per frame (ticks)

/* PID modes */
unsigned int PIDmode;
#define PID_NONE        0
#define PID_SPEED       1

#define FRAME_RATE      33   // frame rate in millis (30Hz)
#define FRAMES          30
unsigned long f_time;        // last frame

unsigned char moving = 0;    // base in motion
unsigned char paused = 0;    // base was in motion, can resume
#define MAXOUTPUT       255  // motor PWM

/* Setpoint Info For a Motor */
typedef struct{
  int Velocity;              // desired actual speed (count/frame)
  long Encoder;              // actual reading
  long PrevEnc;              // last reading
  int PrevErr;
  int Ierror;   
  int output;                // last motor setting
} SetPointInfo;

SetPointInfo left, right;

/* Initialize PID parameters to something known */
void setupPID(){
  // Default values for the PR-MINI
  Kp = 25;
  Kd = 30;
  Ki = 0;
  Ko = 100;
  maxAccel = 50;
  f_time = 0;
}

/* PID control of motor speed */
void DoPid(SetPointInfo * p){
  long Perror;
  long output;
  
  Perror = p->Velocity - (p->Encoder-p->PrevEnc);
          
  // Derivative error is the delta Perror
  output = (Kp*Perror + Kd*(Perror - p->PrevErr) + Ki*p->Ierror)/Ko;
  p->PrevErr = Perror;
  p->PrevEnc = p->Encoder;
  
  output += p->output;   
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAXOUTPUT)
    output = MAXOUTPUT;
  else if (output <= -MAXOUTPUT)
    output = -MAXOUTPUT;
  else
    p->Ierror += Perror;
  
  p->output = output;
}

/* Clear accumulators */
void ClearPID(){
  PIDmode = 0; moving = 0;
  left.PrevErr = 0;
  left.Ierror = 0;
  left.output = 0;
  right.PrevErr = 0;
  right.Ierror = 0;
  right.output = 0;
}

/* This is called by the main loop, does a X HZ PID loop. */
void updatePID(){
  if((moving > 0) && (PIDmode > 0)){  // otherwise, just return
    unsigned long j = millis();
    if(j > f_time){
      // update encoders
      left.Encoder = Encoders.left;
      right.Encoder = Encoders.right;
      // do PID update on PWM
      DoPid(&left);
      DoPid(&right);
      // set updated motor outputs      
      if(PIDmode > 0){
        drive.set(left.output, right.output);
      }
      // update timing
      f_time = j + FRAME_RATE;
    }
  }
}

void clearAll(){
  PIDmode = 0;
  left.Encoder = 0;
  right.Encoder = 0;
  left.PrevEnc = 0;
  right.PrevEnc = 0;
  left.output = 0;
  right.output = 0;
  Encoders.Reset();  
}

