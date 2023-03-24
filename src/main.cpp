#include <Arduino.h>

#include "BlocMoteurs.h"
#include "SpeedController_PP.h"
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

uint32_t timerstart=0;
int8_t updateparam = 0;

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
SpeedController_PP* control;
uint32_t timerBlink;



int rate_ms = 30;



std_msgs::Float32MultiArray wheelSetpoints;
std_msgs::Float32MultiArray readings;
float tab_wheelSetpoints[4] {0.0,0.0,0.0,0.0};
float tab_readings[4] {0.0,0.0,0.0,0.0};

double Vx_setpoint = 0.0;
double Vy_setpoint = 0.0;
 

double Vx_setpoint_not_scaled = 0.0;
double Vy_setpoint_not_scaled = 0.0;


double Vx_setpoint_last = 0.0;
double Vy_setpoint_last = 0.0;
double Vtheta_setpoint_last = 0.0;

double Vtheta_setpoint = 0.0;

double w1,w2,w3,w4 = 0;


char world[] = "map";
char odom[] = "odom";


char tobeprinted[100];



ros::NodeHandle  nh;



void speed_cb( const geometry_msgs::Twist& cmd_msg){

  Vx_setpoint_not_scaled = cmd_msg.linear.x;
  Vy_setpoint_not_scaled = cmd_msg.linear.y;
  



  Vx_setpoint = cmd_msg.linear.x*1000.0;
  Vy_setpoint = cmd_msg.linear.y*1000.0;
  Vtheta_setpoint = cmd_msg.angular.z;

 // if(fabs(Vx_setpoint_last-Vx_setpoint_not_scaled) > 0.2 || fabs(Vy_setpoint_last-Vy_setpoint_not_scaled) > 0.2 )
 // {
 //   control->reset_controller_internal();
 // }



  odo->compute_robot_to_encoders(&Vx_setpoint,&Vy_setpoint,&Vtheta_setpoint,&w1,&w2,&w3,&w4);
  control->define_setpoint(w1,w2,w3,w4);
  wheelSetpoints.data[0] = w1;
  wheelSetpoints.data[1] = w2;
  wheelSetpoints.data[2] = w3;
  wheelSetpoints.data[3] = w4;
  if(millis() > timerBlink + 500)
  {
    digitalToggle(LED_BUILTIN);
    timerBlink = millis();
  }

  Vx_setpoint_last = Vx_setpoint_not_scaled;
  Vy_setpoint_last = Vy_setpoint_not_scaled;
  Vtheta_setpoint_last = 0.0;
  
}


void robot_geometry_callback(const std_msgs::Float32MultiArray & robot_geometry)
{
  
  odo->setRayonRoues(robot_geometry.data[0]);
  odo->setL1pL2(robot_geometry.data[1]);

}



void reset_odo_callback(const std_msgs::Bool & reset)
{
  odo->setX(INIT_X);
  odo->setTheta(INIT_THETA);
  odo->setY(INIT_Y);

}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", speed_cb);
ros::Subscriber<std_msgs::Float32MultiArray> sub_robot_geometry("robot_geometry", robot_geometry_callback);
ros::Subscriber<std_msgs::Bool> sub_reset_odo("reset_odo",reset_odo_callback);
ros::Publisher pub("nav_msgs/odo", &odo_ros);
// ros::Publisher pub_imu("sensor_msgs/Imu", &imu_ros);
  // ros::Publisher pub_setpoint_wheel_speeds("calib/setpoint_wheels", &wheelSetpoints);
  // ros::Publisher pub_wheel_speeds("calib/wheel_speeds", &readings);
tf::TransformBroadcaster odom_broadcaster;
tf::TransformBroadcaster lidar_broadcaster;
// geometry_msgs::TransformStamped lidar_trans;




void setup() {
    

  // Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  wheelSetpoints.data_length=4;
  readings.data_length=4;
  wheelSetpoints.data = tab_wheelSetpoints;
  readings.data = tab_readings;
  timerstart = millis();
  
  


  Wire.begin();
  Wire.setClock(400000);


  // AccGyr.begin();
  // AccGyr.ACC_Enable();  
  // AccGyr.GYRO_Enable();
  // AccGyr.ACC_SetFullScale(ISM330DHCX_16g);
 

  odom_broadcaster.init(nh);
  lidar_broadcaster.init(nh);
  timerBlink = millis();
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  mot = new BlocMoteurs();
  printVar = 0;
  encoder =  new Encoder4Mot();
  odo = new Odometrie(1000*rate_ms,encoder);
  control = new SpeedController_PP(mot,encoder);




  

  nh.subscribe(sub);




  nh.subscribe(sub_robot_geometry);
  nh.subscribe(sub_reset_odo);
  
  nh.advertise(pub);
  // nh.advertise(pub_imu);
  
  // nh.advertise(pub_setpoint_wheel_speeds);
  // nh.advertise(pub_wheel_speeds);
  timer = millis();
  
  
  // geometry_msgs::Quaternion lidar_quat = tf::createQuaternionFromYaw(PI);
  // lidar_trans.header.frame_id = "base_link";

  // lidar_trans.child_frame_id = "laser";
  // lidar_trans.transform.rotation = lidar_quat;

 





}

void loop() {


 
  if(odo->update())
  {
    control->update_controller(false,false);
    // int32_t accelerometer[3];
    // int32_t gyroscope[3];
    
    // AccGyr.ACC_GetAxes(accelerometer);  
    // AccGyr.GYRO_GetAxes(gyroscope);
    ros::Time current_time = nh.now();


    geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(odo->getThetaRadian());

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link_raw";

    odom_trans.transform.translation.x = odo->getX()/1000.0;
    odom_trans.transform.translation.y = odo->getY()/1000.0;
    odom_trans.transform.translation.z = 0.0;   
    odom_trans.transform.rotation = odom_quat;
    
    // lidar_trans.header.stamp= current_time;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    // lidar_broadcaster.sendTransform(lidar_trans);

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

    // imu_ros.angular_velocity.z = (PI*((float)gyroscope[2]))/(180.0*1000.0)    - offset_imu_vel_Z;
    // imu_ros.linear_acceleration.x = (gravity)*((float)accelerometer[0])/(1000.0) - offset_imu_acc_X;
    // imu_ros.linear_acceleration.y = (gravity)*((float)accelerometer[1])/(1000.0) - offset_imu_acc_Y;
    // imu_ros.header.stamp = current_time;
    // imu_ros.header.frame_id = "base_link";
    


    pub.publish(&odom);
    // pub_imu.publish(&imu_ros);
    // pub_setpoint_wheel_speeds.publish(&wheelSetpoints);
    // std::vector<double> toprint = encoder->GetSpeeds();
    // readings.data[0] = toprint[0];
    // readings.data[1] = toprint[1];
    // readings.data[2] = toprint[2];
    // readings.data[3] = toprint[3];
    // pub_wheel_speeds.publish(&readings);
    

  }

  

  nh.spinOnce();
   
}






// #include <Arduino.h>
// #include <Wire.h>
// #include <SPI.h>
// #include "BlocMoteurs.h"
// #include "SpeedPIDController.h"
// #include "EtatDynamiqueRobot.h"
// #include "Encoder4Mot.h"
// #include "Odometrie.h"
// #include <cstdio>
// #include <SpeedController_PP.h>

// #include <global.h>






// uint32_t timerstart=0;
// int8_t updateparam = 0;

// bool reset_odo = false;
// bool last_reset_odo = false;
// bool tuning_mode = false;
// std::vector<std::vector<double>> toprint_vector;
// std::vector<std::vector<double>> toprint_vector_2;


// Encoder4Mot* encoder;
// BlocMoteurs* mot;
// Odometrie* odo;
// uint32_t timer;
// int printVar;
// SpeedController_PP* control;


// int rate_ms = 30;



// float tab[100][4];













// void setup() {
    

//   Serial.begin(115200);

//   timerstart = millis();



//   mot = new BlocMoteurs();
//   encoder =  new Encoder4Mot();
//   odo = new Odometrie(1000*rate_ms,encoder);
//   control = new SpeedController_PP(mot,encoder);

//   delay(5000);

 



//   // mot->motors_on();
//   // mot->commande_vitesses(0.3,0.3,0.3,0.3);
//   // delay(1000);
//   // int i = 0;
//   // timer = millis();
//   // while( i < 10)
//   // {
//   //   if(millis() > timer + 30)
//   //   {
//   //     timer = millis();
//   //     std::vector<double> enco = encoder->Encoder4MotUpdate();
//   //     for(int j = 0; j< 4; j++)
//   //     {
//   //       tab[i][j] = enco[j];

//   //     }
//   //     i+=1;
//   //   }
//   // }
//   // mot->commande_vitesses(0.9,0.9,0.9,0.9);
//   // while( i < 100)
//   // {
//   //   if(millis() > timer + 30)
//   //   {
//   //     timer = millis();
//   //     std::vector<double> enco = encoder->Encoder4MotUpdate();
//   //     for(int j = 0; j< 4; j++)
//   //     {
//   //       tab[i][j] = enco[j];

//   //     }
//   //     i+=1;
//   //   }

//   // }
//   // mot->motors_stop();

  
//   // for(int j = 0; j<100; j++)
//   // {


//   //   for(int k = 0; k<4; k++)
//   //   {
//   //     Serial.print(tab[j][k]);
//   //     Serial.print(";");
//   //   }
//   //   if(j<10)
//   //   {
//   //     Serial.print("1;");
//   //   }
//   //   else
//   //   {
//   //     Serial.print("10;");
//   //   }
//   //   Serial.println(j);
    
    
//   // }

 
//   control->define_setpoint(3.0,3.0,3.0,3.0);
//   timer = millis();
//   while(millis() < timer + 5000)
//   {
//     if(control->update_controller())
//     {
//       // std::valarray<double> toprint = control->getmotors_input();
//       // std::vector<double> toadd {toprint[0],toprint[1],toprint[2],toprint[3]};
//       // toprint_vector.push_back(toadd);
//       // toprint_vector_2.push_back(encoder->GetSpeeds());

//     }

//   }
//   control->define_setpoint(10.0,10.0,10.0,10.0);
//   timer = millis();
//   while(millis() < timer + 5000)
//   {
//     if(control->update_controller())
//     {
//       // std::valarray<double> toprint = control->getmotors_input();
//       // std::vector<double> toadd {toprint[0],toprint[1],toprint[2],toprint[3]};
//       // toprint_vector.push_back(toadd);
//       // toprint_vector_2.push_back(encoder->GetSpeeds());

//     }

//   }
//   control->define_setpoint(1.0,1.0,1.0,1.0);
//   timer = millis();
//   while(millis() < timer + 5000)
//   {
//     if(control->update_controller())
//     {
//       // std::valarray<double> toprint = control->getmotors_input();
//       // std::vector<double> toadd {toprint[0],toprint[1],toprint[2],toprint[3]};
//       // toprint_vector.push_back(toadd);
//       // toprint_vector_2.push_back(encoder->GetSpeeds());

//     }

//   }
//   mot->motors_stop();
//   timer = millis();
//   while(millis() < timer + 5000)
//   {
//     if(control->update_controller())
//     {
//       // std::valarray<double> toprint = control->getmotors_input();
//       // std::vector<double> toadd {toprint[0],toprint[1],toprint[2],toprint[3]};
//       // toprint_vector.push_back(toadd);
//       // toprint_vector_2.push_back(encoder->GetSpeeds());

//     }

//   }
//   mot->motors_stop();
//   // for(int i = 0; i<toprint_vector.size();i++)
//   // {
//   //   for(int k = 0; k<4; k++)
//   //   {
//   //     Serial.print(toprint_vector[i][k]);
//   //     Serial.print(";");
//   //   }
//   //   for(int k = 0; k<4; k++)
//   //   {
//   //     Serial.print(toprint_vector_2[i][k]);
//   //     Serial.print(";");
//   //   }
//   //   Serial.println(i);
    
//   // }


// }

// void loop() {


 

   
// }


















