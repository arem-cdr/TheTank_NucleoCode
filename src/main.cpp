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
#include <sensor_msgs/Imu.h>

#include <Wire.h>
#include <ISM330DHCXSensor.h>

#define dev_interface       Wire
ISM330DHCXSensor AccGyr(&dev_interface);

float offset_imu_acc_X = 0.0;
float offset_imu_acc_Y = 0.0;
float offset_imu_vel_Z = 0.0;

float gravity = 9.80417;

nav_msgs::Odometry odo_ros;
sensor_msgs::Imu imu_ros;

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

char world[] = "map";
char odom[] = "odom";






ros::NodeHandle  nh;



void speed_cb( const geometry_msgs::Twist& cmd_msg){

  Vx_setpoint = cmd_msg.linear.x*1000.0;
  Vy_setpoint = cmd_msg.linear.y*100.0;
  Vtheta_setpoint = cmd_msg.angular.z;
  odo->compute_robot_to_encoders(&Vx_setpoint,&Vy_setpoint,&Vtheta_setpoint,&w1,&w2,&w3,&w4);
  control->define_setpoint(w1,w2,w3,w4);
  if(millis() > timerBlink + 500)
  {
    digitalToggle(LED_BUILTIN);
    timerBlink = millis();
  }
  
}

void parameter_cb(const std_msgs::Bool& update_msg)
{
  nh.getParam("/robot_dynamic_param/wheelRadius_mm",&global_RayonRoues);
  nh.getParam("/robot_dynamic_param/L1pL2_mm",&global_L1pL2);
  nh.getParam("/robot_dynamic_param/offset_imu_acc_X", &offset_imu_acc_X);
  nh.getParam("/robot_dynamic_param/offset_imu_acc_Y",&offset_imu_acc_Y);
  nh.getParam("/robot_dynamic_param/offset_imu_vel_Z",&offset_imu_vel_Z);
  nh.getParam("/robot_dynamic_param/gravity_constant",&gravity);
  odo->setRayonRoues(global_RayonRoues);
  odo->setL1pL2(global_L1pL2);
  if(millis() > timerBlink + 50)
  {
    digitalToggle(LED_BUILTIN);
    timerBlink = millis();
  }
  

}



ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", speed_cb);
ros::Subscriber<std_msgs::Bool> sub2("parameter_update", parameter_cb);
ros::Publisher pub("nav_msgs/odo", &odo_ros);
ros::Publisher pub_imu("sensor_msgs/Imu", &imu_ros);
tf::TransformBroadcaster odom_broadcaster;




void setup() {
    

  // Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  Wire.begin();
  Wire.setClock(400000);


  AccGyr.begin();
  AccGyr.ACC_Enable();  
  AccGyr.GYRO_Enable();


  

  
 

  odom_broadcaster.init(nh);
  timerBlink = millis();
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  mot = new BlocMoteurs();
  delay(3000);
  printVar = 0;
  encoder =  new Encoder4Mot();
  odo = new Odometrie(30000,encoder);
  control = new SpeedPIDController(mot,encoder,30);




	


  

  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.advertise(pub);
  nh.advertise(pub_imu);
  timer = millis();

  
  
 




}

void loop() {


 
  if(odo->update())
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

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

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
  nh.spinOnce();
   
}












