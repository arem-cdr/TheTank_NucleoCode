#include <Arduino.h>
#include "BlocMoteurs.h"
#include "SpeedPIDController.h"
#include "EtatDynamiqueRobot.h"
#include "Encoder4Mot.h"
#include "Odometrie.h"
#include <cstdio>
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

Encoder4Mot* encoder;
BlocMoteurs* mot;
Odometrie* odo;
uint32_t timer;
int printVar;
SpeedPIDController* control;
uint32_t timerBlink;

double Vx_setpoint = 0.0;
double Vy_setpoint = 0.0;
double Vtheta_setpoint = 0.0;

double w1,w2,w3,w4 = 0;

char world[] = "/world>";
char odom[] = "/odom";




void speed_cb( const geometry_msgs::Twist& cmd_msg){

  Vx_setpoint = cmd_msg.linear.x;
  Vy_setpoint = cmd_msg.linear.y;
  Vtheta_setpoint = cmd_msg.angular.z;
  odo->compute_robot_to_encoders(&Vx_setpoint,&Vy_setpoint,&Vtheta_setpoint,&w1,&w2,&w3,&w4);
  control->define_setpoint(w1,w2,w3,w4);
  if(millis() > timerBlink + 500)
  {
    digitalToggle(LED_BUILTIN);
    timerBlink = millis();
  }
  
}

ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", speed_cb);

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;



void setup() {

  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  timerBlink = millis();
  
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  mot = new BlocMoteurs();
  delay(3000);
  printVar = 0;
  encoder =  new Encoder4Mot();
  odo = new Odometrie(30000,encoder);
  control = new SpeedPIDController(mot,encoder,30);

  while(nh.subscribe(sub) != true)
  {
    delay(1000);
  }
  
  broadcaster.init(nh);




}

void loop() {
 
  if(odo->update())
  {
    control->update_controller(false,false);
    t.header.frame_id = world;
    t.child_frame_id = odom;
    t.transform.translation.x = odo->getX();
    t.transform.translation.y = odo->getY();
    t.transform.translation.z = 0.0;
    

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0; 
    t.transform.rotation.z = sin(odo->getThetaRadian()/2); 
    t.transform.rotation.w = cos(odo->getThetaRadian()/2); 

    t.header.stamp = nh.now();
     
  
    broadcaster.sendTransform(t);
    

  }
   nh.spinOnce();
}

/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

// #include <ros.h>
// #include <std_msgs/String.h>

// ros::NodeHandle  nh;

// std_msgs::String str_msg;
// ros::Publisher chatter("chatter", &str_msg);

// char hello[13] = "hello world!";

// void setup()
// {
//   Serial.begin(115200);
//   nh.getHardware()->setBaud(115200);
//   nh.initNode();
//   nh.advertise(chatter);
// }

// void loop()
// {
//   str_msg.data = hello;
//   chatter.publish( &str_msg );
//   nh.spinOnce();
//   delay(1000);
// }
