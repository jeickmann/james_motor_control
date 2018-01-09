#include <PID_v1.h>


#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include "Arduino.h"

ros::NodeHandle nh;

//jimmy settings
//#define TICKS_PER_METER 1130
//int inApin[2] = { 9, 10 };  // INA: Clockwise input
//int inBpin[2] = { 6, 11 }; // INB: Counter-clockwise input
//read second encoder pin for direction (otherwise use current motor setting)
//#define QUAD_ENCODER
//#define ENCODER_A_PIN_R 2
//#define ENCODER_A_PIN_L 3
//#define ENCODER_B_PIN_R 4
//#define ENCODER_B_PIN_L 5

//james settings
//set if we have a separate PWM-PIN
#define TICKS_PER_METER 88
#define THREE_PIN_CONTROL
#define ENCODER_BOTH_FLANKS

const static int inApin[2] = { 8, 4 };  // INA: Clockwise input 
const static int inBpin[2] = { 7, 9 }; // INB: Counter-clockwise input
const static int pwmpin[2] = { 5, 6 }; // PWM input
#define ENCODER_A_PIN_R 3
#define ENCODER_A_PIN_L 2

unsigned long last_cmd = 0;

float maxPower = 0.5;

double vtarget_l = 0;
double cmd_l = 0;
volatile double vel_l = 0;
volatile int lticks;
volatile unsigned long last_time_l;
std_msgs::Int32 lticksMsg;
std_msgs::Float64 lVelMsg;

PID pid_l(&vel_l, &cmd_l, &vtarget_l, 0.5, 0.5, 0, DIRECT);

void lVelCmd(const std_msgs::Float32& msg) {
  last_cmd = millis();
  vtarget_l = msg.data;
  if(vtarget_l < 0) {
    pid_l.SetOutputLimits(-maxPower, 0);
  } else {
      pid_l.SetOutputLimits(0, maxPower);
  }
}

void lTick() {
  #ifdef QUAD_ENCODER
    int dir = digitalRead(encoderBPin[1])==HIGH?1:-1;
  #else
    int dir = (vtarget_l<0)?-1:1;
  #endif
  lticks += dir;

  unsigned long dT = micros() - last_time_l;
  last_time_l = micros();
  vel_l = 1000000 / (float)dT / TICKS_PER_METER * (float)dir;
}

double vtarget_r = 0;
double cmd_r = 0;
volatile double vel_r = 0;
volatile int rticks;
volatile unsigned long last_time_r;
std_msgs::Int32 rticksMsg;
std_msgs::Float64 rVelMsg;

PID pid_r(&vel_r, &cmd_r, &vtarget_r, 0.5, 0.5, 0, DIRECT);

void rVelCmd(const std_msgs::Float32& msg) {
  last_cmd = millis();
  vtarget_r = msg.data;
  if(vtarget_r < 0) {
    pid_r.SetOutputLimits(-maxPower, 0);
  } else {
      pid_r.SetOutputLimits(0, maxPower);
  }
}

void rTick() {
  #ifdef QUAD_ENCODER
    int dir = digitalRead(encoderBPin[0])==HIGH?-1:1;
  #else
    int dir = (vtarget_r<0)?-1:1;
  #endif
  rticks += dir;

  unsigned long dT = micros() - last_time_r;
  last_time_r = micros();
  vel_r = 1000000 / (float)dT / TICKS_PER_METER * (float)dir;
}

ros::Subscriber<std_msgs::Float32> subL("lwheel_vtarget", &lVelCmd);
ros::Subscriber<std_msgs::Float32> subR("rwheel_vtarget", &rVelCmd);
ros::Publisher lTickPub("lticks", &lticksMsg);
ros::Publisher lVelPub("lwheel_vel", &lVelMsg);
ros::Publisher rTickPub("rticks", &rticksMsg);
ros::Publisher rVelPub("rwheel_vel", &rVelMsg);

void initMotor(int number) {
  pinMode(inApin[number], OUTPUT);
  pinMode(inBpin[number], OUTPUT);
  //brake initially
  digitalWrite(inApin[number], LOW);
  digitalWrite(inBpin[number], LOW);
  
  #ifdef THREE_PIN_CONTROL
    pinMode(pwmpin[number], OUTPUT);
    digitalWrite(pwmpin[number], LOW);
  #endif

  pinMode(13, OUTPUT);
}

int oldPower[2] =  {0,0};

void setMotorPower(int number, double power) {
  if(power < 0.1) {
      power = 0;
  }
  int pwmPower = power * 255;
  
  #ifdef THREE_PIN_CONTROL
    if(pwmPower != 0 && oldPower[number] * pwmPower <= 0) {
        digitalWrite(inApin[number], power>0?HIGH:LOW);
        digitalWrite(inBpin[number], power>0?LOW:HIGH);
        oldPower[number] = pwmPower;
    }
    
    analogWrite(pwmpin[number], abs(pwmPower));
  #else
      if (abs(power) < 0.001) {
        digitalWrite(inApin[number], LOW);
        digitalWrite(inBpin[number], LOW);

      } else if (power > 0) {
        pwmPower = 255 - pwmPower;
        analogWrite(inApin[number], pwmPower);
        digitalWrite(inBpin[number], HIGH);
      } else {
        analogWrite(inApin[number], -pwmPower);
        digitalWrite(inBpin[number], LOW);
      }
  #endif
}


//The setup function is called once  at startup of the sketch
void setup() {
  //Serial.begin(115200);
  initMotor(0);
  initMotor(1);
  setMotorPower(0, 0);
  setMotorPower(1, 0);

  lticks = 0;
  pinMode(ENCODER_A_PIN_L, INPUT_PULLUP);
  #ifdef QUAD_ENCODER
    pinMode(ENCODER_B_PIN_L, INPUT_PULLUP);
  #endif
  #ifdef ENCODER_BOTH_FLANKS
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_L), &lTick, CHANGE);
  #else
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_L), &lTick, RISING);
  #endif
  
  rticks = 0;
  pinMode(ENCODER_A_PIN_R, INPUT_PULLUP);
  #ifdef QUAD_ENCODER
    pinMode(ENCODER_B_PIN_R, INPUT_PULLUP);
  #endif
  #ifdef ENCODER_BOTH_FLANKS
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_R), &lTick, CHANGE);
  #else
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_R), &rTick, RISING);
  #endif

  nh.initNode();

  while(!nh.connected()) {nh.spinOnce();}

  int sampleTime = 10;

  float KP = 0.5;
  float KI = 0.5;
  float KD = 0.0;
  nh.getParam("~maxpower", &maxPower);
  nh.getParam("~sampletime", &sampleTime);
  nh.getParam("~Kp_l", &KP);
  nh.getParam("~Ki_l", &KI);
  nh.getParam("~Kd_l", &KD);

  pid_l.SetTunings(KP, KI, KD);
  pid_l.SetSampleTime(sampleTime);
  pid_l.SetOutputLimits(-maxPower, maxPower);
  pid_l.SetMode(AUTOMATIC);
  
  KP = 0.5;
  KI = 0.5;
  KD = 0.0;
  nh.getParam("~Kp_r", &KP);
  nh.getParam("~Ki_r", &KI);
  nh.getParam("~Kd_r", &KD);

  pid_r.SetTunings(KP, KI, KD);
  pid_r.SetSampleTime(sampleTime);
  pid_r.SetOutputLimits(-maxPower, maxPower);
  pid_r.SetMode(AUTOMATIC);
  
  nh.advertise(lTickPub);
  nh.advertise(rTickPub);
  nh.advertise(lVelPub);
  nh.advertise(rVelPub);
  nh.subscribe(subL);
  nh.subscribe(subR);

  digitalWrite(13, LOW);
}

unsigned long nextSpin = 0;
#define SPIN_RATE 10

// The loop function is called in an endless loop
void loop() {
  if((millis() - last_cmd) > 1000) {
    vtarget_l = 0;
    vtarget_r = 0;
    cmd_l = 0;
    cmd_r = 0;
  }
  
  unsigned long dT = micros() - last_time_l;
  if(dT > 500000L) {
    vel_l = 0;
  }

  dT = micros() - last_time_r;
  if(dT > 500000L) {
    vel_r = 0;
  }
  
  if(pid_l.Compute()) {
    if(abs(vtarget_l) < 0.01) {
      setMotorPower(1, 0);
    } else {
      setMotorPower(1, cmd_l);
    }
  }
  
  if(pid_r.Compute()) {
    if(abs(vtarget_r) < 0.01) {
      setMotorPower(0, 0);
    } else {
      setMotorPower(0, cmd_r);
    }
  }

  if(millis() > nextSpin) {
    nextSpin = millis() + 1000/SPIN_RATE;
    
    lticksMsg.data = lticks;
    lTickPub.publish(&lticksMsg);
  
    lVelMsg.data = vel_l;
    lVelPub.publish(&lVelMsg);
  
    rticksMsg.data = rticks;
    rTickPub.publish(&rticksMsg);
    
    rVelMsg.data = vel_r;
    rVelPub.publish(&rVelMsg);
    
    nh.spinOnce();
  }
}
