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
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <global.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>

#include <Wire.h>
#include <ISM330DHCXSensor.h>

#define dev_interface       Wire
ISM330DHCXSensor AccGyr(&dev_interface);

float offset_imu_acc_X = -0.0008;
float offset_imu_acc_Y = -0.206;
float offset_imu_vel_Z = -0.003;

float gravity = 9.80417;

bool reset_odo = false;
bool last_reset_odo = false;
bool tuning_mode = false;

nav_msgs::Odometry odo_ros;
sensor_msgs::Imu imu_ros;

Encoder4Mot* encoder;
BlocMoteurs* mot;
Odometrie* odo;
uint32_t timer;
int printVar;
SpeedPIDController* control;
uint32_t timerBlink;


float ku_m1_fd=0.12;
float ku_m2_fg=0.12;
float ku_m3_bd=0.12;
float ku_m4_bg=0.12;

int rate_ms = 30;

bool m1_enable = true;
bool m2_enable = true;
bool m3_enable = true;
bool m4_enable = true;

std_msgs::Float32MultiArray wheelSetpoints;
std_msgs::Float32MultiArray readings;
float tab_wheelSetpoints[4] {0.0,0.0,0.0,0.0};
float tab_readings[4] {0.0,0.0,0.0,0.0};

double Vx_setpoint = 0.0;
double Vy_setpoint = 0.0;
double Vtheta_setpoint = 0.0;

double w1,w2,w3,w4 = 0;


char world[] = "map";
char odom[] = "odom";






ros::NodeHandle  nh;



void speed_cb( const geometry_msgs::Twist& cmd_msg){


  Vx_setpoint = cmd_msg.linear.x*1000.0;
  Vy_setpoint = cmd_msg.linear.y*1000.0;
  Vtheta_setpoint = cmd_msg.angular.z;
  odo->compute_robot_to_encoders(&Vx_setpoint,&Vy_setpoint,&Vtheta_setpoint,&w1,&w2,&w3,&w4);
  control->define_setpoint(w1*m1_enable,w2*m2_enable,w3*m3_enable,w4*m4_enable);
  if(millis() > timerBlink + 500)
  {
    digitalToggle(LED_BUILTIN);
    timerBlink = millis();
  }


  
}

void parameter_cb(const std_msgs::Int8 & update_msg)
{
  if( update_msg.data == 0)
  {
   
    nh.getParam("/robot_dynamic_param/wheelRadius_mm",&global_RayonRoues);
    odo->setRayonRoues(global_RayonRoues);
  }
  else if( update_msg.data == 1)
  {
   
    nh.getParam("/robot_dynamic_param/L1pL2_mm",&global_L1pL2);
    odo->setL1pL2(global_L1pL2);
  }
  else if( update_msg.data == 2)
  {
   
    nh.getParam("/robot_dynamic_param/offset_imu_acc_X", &offset_imu_acc_X);
  }
  else if( update_msg.data == 3)
  {
   
    nh.getParam("/robot_dynamic_param/offset_imu_acc_Y",&offset_imu_acc_Y);
  }
  else if( update_msg.data == 4)
  {
   
    nh.getParam("/robot_dynamic_param/offset_imu_vel_Z",&offset_imu_vel_Z);
  }
  else if( update_msg.data == 5)
  {
   
    nh.getParam("/robot_dynamic_param/gravity_constant",&gravity);
  }
  else if( update_msg.data == 6)
  {
   
    nh.getParam("/robot_dynamic_param/reset_odo",&reset_odo);
    odo->setX(INIT_X);
    odo->setY(INIT_Y);
    odo->setTheta(INIT_THETA);
    last_reset_odo = reset_odo;
  }
  else if( update_msg.data == 7)
  {
   
    nh.getParam("/robot_dynamic_param/tuning_mode",&tuning_mode);
    if(tuning_mode == true)
    {
      control->set_calib(ku_m1_fd,ku_m2_fg,ku_m3_bd,ku_m4_bg);
    }
    else
    {
      control->unset_calib(ku_m1_fd,ku_m2_fg,ku_m3_bd,ku_m4_bg);
    }
  }
  else if( update_msg.data == 8)
  {
   
    nh.getParam("robot_dynamic_param/Ku_m1_fd",&ku_m1_fd);
  }
  else if( update_msg.data == 9)
  {
   
    nh.getParam("robot_dynamic_param/Ku_m2_fg",&ku_m2_fg);
  }
  else if( update_msg.data == 10)
  {
   
    nh.getParam("robot_dynamic_param/Ku_m3_bd",&ku_m3_bd);
  }
  else if( update_msg.data == 11)  
  {
   
    nh.getParam("robot_dynamic_param/Ku_m4_bg",&ku_m4_bg);
  }
  else if( update_msg.data == 12)  
  {
   
    nh.getParam("robot_dynamic_param/m1_enable",&m1_enable);
  }
  else if( update_msg.data == 13)  
  {
   
    nh.getParam("robot_dynamic_param/m2_enable",&m2_enable);
  }
  else if( update_msg.data == 14)  
  {
   
    nh.getParam("robot_dynamic_param/m3_enable",&m3_enable);
  }
  else if( update_msg.data == 15)  
  {
   
    nh.getParam("robot_dynamic_param/m4_enable",&m4_enable);
  }
  else if( update_msg.data == 16)  
  {
   
    nh.getParam("robot_dynamic_param/rate_ms",&rate_ms);
    odo->set_min_update_period_us(1000*rate_ms);
    control->set_rate(rate_ms);
 }


  
  if(millis() > timerBlink + 50)
  {
    digitalToggle(LED_BUILTIN);
    timerBlink = millis();
  }
  

}



ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", speed_cb);
ros::Subscriber<std_msgs::Int8> sub2("parameter_update", parameter_cb);
ros::Publisher pub("nav_msgs/odo", &odo_ros);
ros::Publisher pub_imu("sensor_msgs/Imu", &imu_ros);
ros::Publisher pub_setpoint_wheel_speeds("calib/setpoint_wheels", &wheelSetpoints);
ros::Publisher pub_wheel_speeds("calib/wheel_speeds", &readings);
tf::TransformBroadcaster odom_broadcaster;
tf::TransformBroadcaster lidar_broadcaster;
geometry_msgs::TransformStamped lidar_trans;




void setup() {
    

  // Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  wheelSetpoints.data_length=4;
  readings.data_length=4;
  wheelSetpoints.data = tab_wheelSetpoints;
  readings.data = tab_readings;
  
  


  Wire.begin();
  Wire.setClock(400000);


  AccGyr.begin();
  AccGyr.ACC_Enable();  
  AccGyr.GYRO_Enable();
  AccGyr.ACC_SetFullScale(ISM330DHCX_16g);
  
  // std_msgs::Bool dummyMsg;
  // dummyMsg.data = true;
  // parameter_cb(dummyMsg);

  
 

  odom_broadcaster.init(nh);
  lidar_broadcaster.init(nh);
  timerBlink = millis();
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  mot = new BlocMoteurs();
  delay(3000);
  printVar = 0;
  encoder =  new Encoder4Mot();
  odo = new Odometrie(1000*rate_ms,encoder);
  control = new SpeedPIDController(mot,encoder,rate_ms);




	


  

  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.advertise(pub);
  nh.advertise(pub_imu);
  
  nh.advertise(pub_setpoint_wheel_speeds);
  nh.advertise(pub_wheel_speeds);
  timer = millis();
  
  
  geometry_msgs::Quaternion lidar_quat = tf::createQuaternionFromYaw(PI);
  lidar_trans.header.frame_id = "base_link";

  lidar_trans.child_frame_id = "laser";
  lidar_trans.transform.rotation = lidar_quat;

  
  
 




}

void loop() {


 
  if(odo->update() && tuning_mode == false)
  {
    control->update_controller(false,false);
    int32_t accelerometer[3];
    int32_t gyroscope[3];
    
    AccGyr.ACC_GetAxes(accelerometer);  
    AccGyr.GYRO_GetAxes(gyroscope);
    ros::Time current_time = nh.now();


    geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(odo->getThetaRadian());

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = odo->getX()/1000.0;
    odom_trans.transform.translation.y = odo->getY()/1000.0;
    odom_trans.transform.translation.z = 0.0;   
    odom_trans.transform.rotation = odom_quat;
    
    lidar_trans.header.stamp= current_time;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    lidar_broadcaster.sendTransform(lidar_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = odo->getX()/1000.0;
    odom.pose.pose.position.y = odo->getY()/1000.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = odo->getVXEnco()/1000.0;
    odom.twist.twist.linear.y = odo->getVYEnco()/1000.0;
    odom.twist.twist.angular.z = odo->getVThetaEnco();

    imu_ros.angular_velocity.z = (PI*((float)gyroscope[2]))/(180.0*1000.0)    - offset_imu_vel_Z;
    imu_ros.linear_acceleration.x = (gravity)*((float)accelerometer[0])/(1000.0) - offset_imu_acc_X;
    imu_ros.linear_acceleration.y = (gravity)*((float)accelerometer[1])/(1000.0) - offset_imu_acc_Y;
    imu_ros.header.stamp = current_time;
    imu_ros.header.frame_id = "base_link";
    


    pub.publish(&odom);
    pub_imu.publish(&imu_ros);
    

  }
  else if (tuning_mode)
  {
    if(control->update_controller())
    {
      tab_wheelSetpoints[0] = (float) w1;
      tab_wheelSetpoints[1] = (float) w2;
      tab_wheelSetpoints[2] = (float) w3;
      tab_wheelSetpoints[3] = (float) w4;
      std::vector<double> toprint = encoder->GetSpeeds();
      for(int i = 0; i< 4; i++)
      {
        tab_readings[i] = toprint[i];
      }

      pub_setpoint_wheel_speeds.publish(&wheelSetpoints);
      pub_wheel_speeds.publish(&readings);
      
      int32_t accelerometer[3];
      int32_t gyroscope[3];
    
      AccGyr.ACC_GetAxes(accelerometer);  
      AccGyr.GYRO_GetAxes(gyroscope);
      ros::Time current_time = nh.now();


      geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(odo->getThetaRadian());

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = odo->getX()/1000.0;
      odom_trans.transform.translation.y = odo->getY()/1000.0;
      odom_trans.transform.translation.z = 0.0;   
      odom_trans.transform.rotation = odom_quat;
      
      lidar_trans.header.stamp= current_time;

      //send the transform
      odom_broadcaster.sendTransform(odom_trans);
      lidar_broadcaster.sendTransform(lidar_trans);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = odo->getX()/1000.0;
      odom.pose.pose.position.y = odo->getY()/1000.0;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = odo->getVXEnco()/1000.0;
      odom.twist.twist.linear.y = odo->getVYEnco()/1000.0;
      odom.twist.twist.angular.z = odo->getVThetaEnco();

      imu_ros.angular_velocity.z = (PI*((float)gyroscope[2]))/(180.0*1000.0)    - offset_imu_vel_Z;
      imu_ros.linear_acceleration.x = (gravity)*((float)accelerometer[0])/(1000.0) - offset_imu_acc_X;
      imu_ros.linear_acceleration.y = (gravity)*((float)accelerometer[1])/(1000.0) - offset_imu_acc_Y;
      imu_ros.header.stamp = current_time;
      imu_ros.header.frame_id = "base_link";
      

      pub.publish(&odom);
      pub_imu.publish(&imu_ros);
      odom_broadcaster.sendTransform(odom_trans);
      lidar_broadcaster.sendTransform(lidar_trans);
      
    }

  }
  nh.spinOnce();
   
}












