#include "Motor.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include<PID_v1.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>

//************************
//Değiştirilecek bölüm
const float pulsesperturn = 20.0;
const float wheel_diameter = 0.66, wheel_separation = 0.8; //In meters//const float wheel_diameter = 0.066, wheel_separation = 0.08; //In meters
Motor right(PA6,PA7,PB0); //Pin forward, pin backward, pin encoder
Motor left(PA1,PA2,PA3);  //Pin forward, pin backward, pin encoder
//********************


const float distancePerPulse = (PI * wheel_diameter) / pulsesperturn;

ros::NodeHandle  nh;

#define LOOPTIME 10

unsigned int left_pulses = 0, right_pulses = 0;

double left_kp = 3.8, left_ki = 0.0, left_kd = 0.0;             // modify for optimal performance
double right_kp = 0.0, right_ki = 0.0, right_kd = 0.0;

double right_input = 0.0, right_output = 0.0, right_setpoint = 0.0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0.0, left_output = 0.0, left_setpoint = 0.0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

float demandx=0.0;
float demandz=0.0;

double demand_speed_left;
double demand_speed_right;


unsigned long currentMillis;
unsigned long prevMillis;

float leftdiff;
float rightdiff;

float encoder0Error;
float encoder1Error;

float left_pulses_prev;
float right_pulses_prev;

void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

double speed_act_left = 0.0;                    //Actual speed for left wheel in m/s
double speed_act_right = 0.0;                    //Command speed for left wheel in m/s 

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
//  Serial.begin(115200);
  
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-100, 100);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-100, 100);

  attachInterrupt(digitalPinToInterrupt(left.encoder), change_left, FALLING);
  attachInterrupt(digitalPinToInterrupt(right.encoder), change_right, FALLING);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME){
    
    prevMillis = currentMillis;

    demand_speed_left = demandx - (demandz*wheel_separation);
    demand_speed_right = demandx + (demandz*wheel_separation);
    
    leftdiff = left_pulses - left_pulses_prev;
    rightdiff = right_pulses - right_pulses_prev;
    
    speed_act_left = leftdiff/distancePerPulse;                    
    speed_act_right = rightdiff/distancePerPulse; 
  
    encoder0Error = (demand_speed_left*distancePerPulse)-leftdiff;
    encoder1Error = (demand_speed_right*distancePerPulse)-rightdiff;
  
    left_pulses_prev = left_pulses;
    right_pulses_prev = right_pulses;
  
    left_setpoint = demand_speed_left*distancePerPulse;
    right_setpoint = demand_speed_right*distancePerPulse;
  
    left_input = leftdiff;
    right_input = rightdiff;
    
    leftPID.Compute();
    left.rotate(left_output);
    rightPID.Compute();
    right.rotate(right_output);
  }
  publishSpeed(LOOPTIME);
  nh.spinOnce();
}


//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
}


// ************** encoders interrupts **************

void change_left(){
  if (left.wheelDirection > 0){
    left_pulses++;
  }
  else{
    left_pulses--;
  } 
  
   
}

void change_right(){
  if (right.wheelDirection > 0) {
    right_pulses++;
  }
  else{
    right_pulses--;  
  } 
}
