#include <Arduino.h>
#include "BlocMoteurs.h"
#include "SpeedPIDController.h"
#include "EtatDynamiqueRobot.h"
#include "Encoder4Mot.h"
#include "Odometrie.h"
#include <cstdio>
// #include <ros.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <ros/time.h>
// #include <tf/transform_broadcaster.h>

Encoder4Mot* encoder;
BlocMoteurs* mot;
Odometrie* odo;
uint32_t timer;
int printVar;
SpeedPIDController* control;

double Vx_setpoint = 0.0;
double Vy_setpoint = 0.0;
double Vtheta_setpoint = 0.0;

double w1,w2,w3,w4 = 0;

// char base_link[] = "/base_link";
// char odom[] = "/odom";


// void speed_cb( const std_msgs::Float64MultiArray& cmd_msg){
//   Vx_setpoint = cmd_msg.data[0];
//   Vy_setpoint = cmd_msg.data[1];
//   Vtheta_setpoint = cmd_msg.data[2];
//   odo->compute_robot_to_encoders(&Vx_setpoint,&Vy_setpoint,&Vtheta_setpoint,&w1,&w2,&w3,&w4);
//   control->define_setpoint(w1,w2,w3,w4);
// }

// ros::NodeHandle  nh;
// ros::Subscriber<std_msgs::Float64MultiArray> sub("robot_speed", speed_cb);

// geometry_msgs::TransformStamped t;
// tf::TransformBroadcaster broadcaster;



void setup() {
  
  // nh.initNode();
  // broadcaster.init(nh);

  mot = new BlocMoteurs();
  delay(3000);
  printVar = 0;
  encoder =  new Encoder4Mot();
  odo = new Odometrie(30000,encoder);
  control = new SpeedPIDController(mot,encoder,30);
  // nh.subscribe(sub);

  control->define_setpoint(3.0,3.0,3.0,3.0);
  timer = millis();
  uint32_t timerAsser = millis();
  while( millis() < timer + 10000)
  {
    if(control->update_controller())
    {
      std::vector<double> toprint = encoder->GetSpeeds();
      Serial.print("$ ");
      Serial.print(toprint[0]);
      Serial.print(" ");
      Serial.print(toprint[1]);
      Serial.print(" ");
      Serial.print(toprint[2]);
      Serial.print(" ");
      Serial.print(toprint[3]);
      Serial.println(" ;");
    }
  }




  while(millis() < timer + 2000)
  {
    
    if(millis() > timerAsser + 30)
    {
      std::vector<double> speeds = encoder->Encoder4MotUpdate();
      Serial.print("$ ");
      Serial.print(speeds[0]);
      Serial.print(" ");
      Serial.print(speeds[1]);
      Serial.print(" ");
      Serial.print(speeds[2]);
      Serial.print(" ");
      Serial.print(speeds[3]);
      Serial.print(" ");
      Serial.print(speeds[3]);
      Serial.println(" 2;");
      timerAsser = millis();
    }

  }
  mot->commande_vitesses(0.3,0.3,0.3,0.3);
  timer = millis();
  timerAsser = millis();
  while(millis() < timer + 2000)
  {
    
    if(millis() > timerAsser + 30)
    {
      std::vector<double> speeds = encoder->Encoder4MotUpdate();
      Serial.print("$ ");
      Serial.print(speeds[0]);
      Serial.print(" ");
      Serial.print(speeds[1]);
      Serial.print(" ");
      Serial.print(speeds[2]);
      Serial.print(" ");
      Serial.print(speeds[3]);
      Serial.println(" 11;");
      timerAsser = millis();
    }

  }



}

void loop() {
 
  // if(odo->update())
  // {
  //   control->update_controller(false,false);
  //   t.header.frame_id = odom;
  //   t.child_frame_id = base_link;
  //   t.transform.translation.x = 1.0; 
  //   t.transform.rotation.x = 0.0;
  //   t.transform.rotation.y = 0.0; 
  //   t.transform.rotation.z = 0.0; 
  //   t.transform.rotation.w = 1.0;  
  //   t.header.stamp = nh.now();
  //   broadcaster.sendTransform(t);
    

  // }
  //  nh.spinOnce();
}