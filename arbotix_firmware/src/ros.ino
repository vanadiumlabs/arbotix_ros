/* 
  ArbotiX Firmware for ROS driver
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

/* Build Configuration */
#define USE_BASE            // Enable support for a mobile base
#define USE_HW_SERVOS       // Enable only 2/8 servos, but using hardware control

#define CONTROLLER_COUNT    5
/* Hardware Constructs */
#include <ax12.h>
#include <BioloidController.h>
BioloidController controllers[CONTROLLER_COUNT];

#include "ros.h"

#ifdef USE_HW_SERVOS
  #include <HServo.h>
  HServo servos[2];
  int servo_vals[2];            // in millis
#else 
  #include <Servo.h>
  Servo servos[10];
  int servo_vals[10];           // in millis
#endif

#ifdef USE_BASE
  #include <Motors2.h>
  Motors2 drive = Motors2();
  #include <EncodersAB.h>
  #include "diff_controller.h"
#endif

/* Register Storage */
unsigned char baud = 7;         // ?
unsigned char ret_level = 1;    // ?
unsigned char alarm_led = 0;    // ?

/* Pose & Sequence Structures */
typedef struct{
  unsigned char pose;           // index of pose to transition to 
  int time;                     // time for transition
} sp_trans_t;
int poses[30][AX12_MAX_SERVOS]; // poses [index][servo_id-1]
sp_trans_t sequence[50];        // sequence
int seqPos;                     // step in current sequence

#include "user_hooks.h"

/* 
 * Setup Functions
 */
void scan(){
#if defined(AX_RX_SWITCHED)
  // do a search for devices on the RX bus, default to AX if not found
  int i;
  for(i=0;i<AX12_MAX_SERVOS;i++){
    dynamixel_bus_config[i] = 1;
    if(ax12GetRegister(i+1, AX_ID, 1) != (i+1)){
      dynamixel_bus_config[i] = 0;
    }
  }
#endif
}

void setup(){
  Serial.begin(115200);
  ax12Init(1000000);

#ifdef USE_BASE
  drive.init();
  Encoders.Begin();
  setupPID();
#endif

#if defined(AX_RX_SWITCHED)
  delay(1000);
  scan();
#endif

  userSetup();
  pinMode(0,OUTPUT);     // status LED
}

/*
 * Handle Write Requests to ArbotiX Registers 
 */
unsigned char handleWrite(){
  int addr  = params[0];  // address to write
  int bytes = length-3;   // # of bytes left to write
  int k = 1;              // index in parameters of value to write

  while(bytes > 0){
    if(addr < REG_BAUD_RATE){
      return ERR_INSTRUCTION;
    }else if(addr == REG_BAUD_RATE){
      UBRR1L = params[k];
    }else if(addr < REG_RESCAN){
      return ERR_INSTRUCTION; // can't write digital inputs
    }else if(addr == REG_RESCAN){
      scan();
    }else if(addr == REG_RETURN_LEVEL){
      ret_level = params[k];
    }else if(addr == REG_ALARM_LED){
      // TODO: 
    }else if(addr < REG_SERVO_BASE){
      return ERR_INSTRUCTION; // error - analog are read only
    }else if(addr < REG_MOVING){
      // write servo
      int s = addr - REG_SERVO_BASE;
 #ifdef USE_HW_SERVO
      if( s >= 4 ){
 #else
      if( s >= 20){
 #endif 
        return ERR_INSTRUCTION;
      }else{
        if( s%2 == 0 ){ // low byte
          s = s/2;
          servo_vals[s] = params[k];
        }else{          // high byte
          s = s/2;
          servo_vals[s] += (params[k]<<8);
          if(servo_vals[s] > 500 && servo_vals[s] < 2500){
            servos[s].writeMicroseconds(servo_vals[s]);
            if(!servos[s].attached())            
              servos[s].attach(s);
          }else if(servo_vals[s] == 0){
            servos[s].detach();
          }
        }
      }
    }else if(addr == REG_MOVING){
      return ERR_INSTRUCTION;
    }else if(addr < REG_RESERVED){
      // write digital pin
      int pin = addr - REG_DIGITAL_OUT0;
    #ifdef SERVO_STIK
      if(pin < 8)
        pin = pin+24;
      else
        pin = pin+5; // servo stick 8 = D13...
    #endif
      if(params[k] & 0x02)    // high
        digitalWrite(pin, HIGH);
      else
        digitalWrite(pin, LOW);
      if(params[k] & 0x01)    // output
        pinMode(pin, OUTPUT);
      else
        pinMode(pin, INPUT);
    }else{
      int ret = userWrite(addr, params[k]);
      if(ret > ERR_NONE) return ret;
    }
    addr++;k++;bytes--;
  }
  return ERR_NONE;
}


/*
 * Handle a read from ArbotiX registers.
 */
int handleRead(){
  int checksum = 0;
  int addr = params[0];
  int bytes = params[1];
  
  unsigned char v;
  while(bytes > 0){
    if(addr == REG_MODEL_NUMBER_L){ 
      v = 44;
    }else if(addr == REG_MODEL_NUMBER_H){
      v = 1;  // 300 
    }else if(addr == REG_VERSION){
      v = 0;
    }else if(addr == REG_ID){
      v = 253;
    }else if(addr == REG_BAUD_RATE){
      v = 34; // 56700
    }else if(addr == REG_DIGITAL_IN0){
      // digital 0-7
    #ifdef SERVO_STIK
      v = PINA;
    #else
      v = PINB;
    #endif
    }else if(addr == REG_DIGITAL_IN1){
      // digital 8-15
    #ifdef SERVO_STIK
      v = (PINB>>1);
    #else
      v = PIND;
    #endif        
    }else if(addr == REG_DIGITAL_IN2){
      // digital 16-23
      v = PINC;
    }else if(addr == REG_DIGITAL_IN3){
      // digital 24-31
      v = PINA;
    }else if(addr == REG_RETURN_LEVEL){
      v = ret_level;
    }else if(addr == REG_ALARM_LED){
      // TODO
    }else if(addr < REG_SERVO_BASE){
      // send analog reading
      int x = analogRead(addr-REG_ANA_BASE)>>2;
      x += analogRead(addr-REG_ANA_BASE)>>2;
      x += analogRead(addr-REG_ANA_BASE)>>2;
      x += analogRead(addr-REG_ANA_BASE)>>2;
      v = x/4;
    }else if(addr < REG_MOVING){
      // send servo position
      v = 0;      
    }else{
      v = userRead(addr);  
    } 
    checksum += v;
    Serial.write(v);
    addr++;bytes--;
  }
  
  return checksum;
}

int doPlaySeq(){
  seqPos = 0; int i;
  while(sequence[seqPos].pose != 0xff){
    int p = sequence[seqPos].pose;
    // are we HALT?
    if(Serial.read() == 'H') return 1;
    // load pose
    for(i=0; i<controllers[0].poseSize; i++)
      controllers[0].setNextPose(i+1,poses[p][i]); 
    controllers[0].interpolateSetup(sequence[seqPos].time);
    while(controllers[0].interpolating)
      controllers[0].interpolateStep();
    // next transition
    seqPos++;
  }
  return 0;
}

/*
 * Send status packet
 */
void statusPacket(int id, int err){
  Serial.write(0xff);
  Serial.write(0xff);
  Serial.write(id);
  Serial.write(2);
  Serial.write(err);
  Serial.write(255-((id+2+err)%256));
}

/* 
 * decode packets: ff ff id length ins params checksum
 *   same as ax-12 table, except, we define new instructions for Arbotix 
 */
void loop(){
  int i;
    
  // process messages
  while(Serial.available() > 0){
    // We need to 0xFF at start of packet
    if(mode == 0){         // start of new packet
      if(Serial.read() == 0xff){
        mode = 2;
        digitalWrite(0,HIGH-digitalRead(0));
      }
    //}else if(mode == 1){   // another start byte
    //    if(Serial.read() == 0xff)
    //        mode = 2;
    //    else
    //        mode = 0;
    }else if(mode == 2){   // next byte is index of servo
      id = Serial.read();    
      if(id != 0xff)
        mode = 3;
    }else if(mode == 3){   // next byte is length
      length = Serial.read();
      checksum = id + length;
      mode = 4;
    }else if(mode == 4){   // next byte is instruction
      ins = Serial.read();
      checksum += ins;
      index = 0;
      mode = 5;
    }else if(mode == 5){   // read data in 
      params[index] = Serial.read();
      checksum += (int) params[index];
      index++;
      if(index + 1 == length){  // we've read params & checksum
        mode = 0;
        if((checksum%256) != 255){ 
          // return an error packet: FF FF id Len Err=bad checksum, params=None check
          statusPacket(id, ERR_CHECKSUM);
        }else if(id == 253){  // ID = 253, ArbotiX instruction
          switch(ins){     
            case AX_WRITE_DATA:
              // send return packet
              statusPacket(id,handleWrite());
              break;
             
            case AX_READ_DATA:
              checksum = id + params[1] + 2;                            
              Serial.write(0xff);
              Serial.write(0xff);
              Serial.write(id);
              Serial.write((unsigned char)2+params[1]);
              Serial.write((unsigned char)0);
              // send actual data
              checksum += handleRead();
              Serial.write(255-((checksum)%256));
              break;
             
            case ARB_SIZE_POSE:                   // Pose Size = 7, followed by single param: size of pose
              statusPacket(id,0);
              if(controllers[0].poseSize == 0)
                controllers[0].setup(18);
              controllers[0].poseSize = params[0];
              controllers[0].readPose();    
              break;
             
            case ARB_LOAD_POSE:                   // Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size)
              statusPacket(id,0);
              for(i=0; i<controllers[0].poseSize; i++)
                poses[params[0]][i] = params[(2*i)+1]+(params[(2*i)+2]<<8); 
              break;
             
            case ARB_LOAD_SEQ:                    // Load Seq = 9, followed by index/times (# of parameters = 3*seq_size) 
              statusPacket(id,0);
              for(i=0;i<(length-2)/3;i++){
                sequence[i].pose = params[(i*3)];
                sequence[i].time = params[(i*3)+1] + (params[(i*3)+2]<<8);
              }
              break;
             
            case ARB_PLAY_SEQ:                   // Play Seq = A, no params   
              statusPacket(id,0);
              doPlaySeq();
              break;
             
            case ARB_LOOP_SEQ:                   // Play Seq until we recieve a 'H'alt
              statusPacket(id,0);
              while(doPlaySeq() > 0);
              break;

            // ARB_TEST is deprecated and removed

            case ARB_CONTROL_SETUP:              // Setup a controller
              statusPacket(id,0);
              if(params[0] < CONTROLLER_COUNT){
                controllers[params[0]].setup(length-3);
                for(int i=0; i<length-3; i++){
                  controllers[params[0]].setId(i, params[i+1]);
                }
#ifdef USE_BASE
              }else if(params[0] == 10){
                Kp = params[1];
                Kd = params[2];
                Ki = params[3];
                Ko = params[4];
#endif
              }
              break;

            case ARB_CONTROL_WRITE:              // Write values to a controller
              statusPacket(id,0);
              if(params[0] < CONTROLLER_COUNT){
                for(int i=0; i<length-4; i+=2){
                  controllers[params[0]].setNextPose(controllers[params[0]].getId(i/2), params[i+1]+(params[i+2]<<8));
                }
                controllers[params[0]].readPose();
                controllers[params[0]].interpolateSetup(params[length-3]*33);
#ifdef USE_BASE
              }else if(params[0] == 10){
                left_speed = params[1];
                left_speed += (params[2]<<8);
                right_speed = params[3];
                right_speed += (params[4]<<8); 
                if((left_speed == 0) && (right_speed == 0)){
                  drive.set(0,0);
                  ClearPID();
                }else{
                  if((left.Velocity == 0) && (right.Velocity == 0)){
                    PIDmode = 1; moving = 1;
                    left.PrevEnc = Encoders.left;
                    right.PrevEnc = Encoders.right;
                  }
                }   
                left.Velocity = left_speed;
                right.Velocity = right_speed; 
#endif
              }
              break;

            case ARB_CONTROL_STAT:               // Read status of a controller
              if(params[0] < CONTROLLER_COUNT){             
                Serial.write((unsigned char)0xff);
                Serial.write((unsigned char)0xff);
                Serial.write((unsigned char)id);
                Serial.write((unsigned char)3);
                Serial.write((unsigned char)0);
                checksum = controllers[params[0]].interpolating;
                Serial.write((unsigned char)checksum);
                checksum += id + 3;
                Serial.write((unsigned char)255-((checksum)%256));
#ifdef USE_BASE
              }else if(params[0] == 10){
                checksum = id + 2 + 8;                            
                Serial.write((unsigned char)0xff);
                Serial.write((unsigned char)0xff);
                Serial.write((unsigned char)id);
                Serial.write((unsigned char)2+8);
                Serial.write((unsigned char)0);   // error level
                int v = ((unsigned long)Encoders.left>>0)%256;
                Serial.write((unsigned char)v);
                checksum += v;
                v = ((unsigned long)Encoders.left>>8)%256;
                Serial.write((unsigned char)v);
                checksum += v;
                v = ((unsigned long)Encoders.left>>16)%256;
                Serial.write((unsigned char)v);
                checksum += v;
                v = ((unsigned long)Encoders.left>>24)%256;
                Serial.write((unsigned char)v);
                checksum += v;
                v = ((unsigned long)Encoders.right>>0)%256;
                Serial.write((unsigned char)v);
                checksum += v;
                v = ((unsigned long)Encoders.right>>8)%256;
                Serial.write((unsigned char)v);
                checksum += v;
                v = ((unsigned long)Encoders.right>>16)%256;
                Serial.write((unsigned char)v);
                checksum += v;
                v = ((unsigned long)Encoders.right>>24)%256;
                Serial.write((unsigned char)v);
                checksum += v;
                Serial.write((unsigned char)255-((checksum)%256));
#endif
              }
              break;
          }
        }else if(id == 0xFE){
          // sync read or write
          if(ins == ARB_SYNC_READ){
            int start = params[0];    // address to read in control table
            int bytes = params[1];    // # of bytes to read from each servo
            int k = 2;
            checksum = id + (bytes*(length-4)) + 2;                            
            Serial.write((unsigned char)0xff);
            Serial.write((unsigned char)0xff);
            Serial.write((unsigned char)id);
            Serial.write((unsigned char)2+(bytes*(length-4)));
            Serial.write((unsigned char)0);     // error code
            // send actual data
            for(k=2; k<length-2; k++){
              if( ax12GetRegister(params[k], start, bytes) >= 0){
                for(i=0;i<bytes;i++){
                  checksum += ax_rx_buffer[5+i];
                  Serial.write((unsigned char)ax_rx_buffer[5+i]);
                }
              }else{
                for(i=0;i<bytes;i++){
                  checksum += 255;
                  Serial.write((unsigned char)255);
                }
              }
            }
            Serial.write((unsigned char)255-((checksum)%256));
          }else{    
            // TODO: sync write pass thru
            int k;
            setTXall();
            ax12write(0xff);
            ax12write(0xff);
            ax12write(id);
            ax12write(length);
            ax12write(ins);
            for(k=0; k<length; k++)
                ax12write(params[k]);
            // no return
          }       
        }else{ // ID != 253, pass thru 
          switch(ins){
            // TODO: streamline this
            case AX_READ_DATA:
              ax12GetRegister(id, params[0], params[1]);
              // return a packet: FF FF id Len Err params check
              if(ax_rx_buffer[3] > 0){
                for(i=0;i<ax_rx_buffer[3]+4;i++)
                  Serial.write(ax_rx_buffer[i]);
              }
              ax_rx_buffer[3] = 0;
              break;
             
            case AX_WRITE_DATA:
              if(length == 4){
                ax12SetRegister(id, params[0], params[1]);
              }else{
                int x = params[1] + (params[2]<<8);
                ax12SetRegister2(id, params[0], x);
              }
              statusPacket(id,0);
              break;
             
          }
        }
      }
    } // end mode == 5
  } // end while(available)
  // update joints
  for(int i=0; i<5; i++)
    controllers[i].interpolateStep();
 
#ifdef USE_BASE
  // update pid
  updatePID();
#endif

}
