#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <sstream>

#define  setbit(x, y)  x|=(1<<y)
#define  clrbit(x, y)  x&=~(1<<y)


//global variable
geometry_msgs::Twist twist_cmd;
ros::Publisher twist_pub;

const double warn_range = 0.3;  //warn check distance
double default_period_hz = 10;  //hz
double default_linear_x = 0.5;  // (m/s)
double default_yaw_rate = 0.5;  // rad/s

double range_array[3]; //save three obstacle distances
double des_cmd_array[2]; //save the linear_x and angular_z from des_cmd


void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{ 
  range_array[0] = 100;
  range_array[1] = 100;
  range_array[2] = 100; //reset the obstacle distances to 100

  for(int i = 60; i < 90; i++) //find the cloest obstacle on right
  {
    if (msg->ranges[i] < range_array[0]) range_array[0] = msg->ranges[i];
    else continue;
  }

  for(int i = 105; i < 165; i++) //find the cloest obstacle ahead
  {
    if (msg->ranges[i] < range_array[1]) range_array[1] = msg->ranges[i];
    else continue;
  }

  for(int i = 180; i < 210; i++) //find the cloest obstacle on left
  {
    if (msg->ranges[i] < range_array[2]) range_array[2] = msg->ranges[i];
    else continue;
  }
  
}

void des_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  des_cmd_array[0] = msg->linear.x;
  des_cmd_array[1] = msg->angular.z;
}

void publishTwistCmd(double linear_x, double angular_z)
{
    twist_cmd.linear.x = linear_x;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
 
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = angular_z;
 
    twist_pub.publish(twist_cmd);
}

void rosmyrobot(double lidar_r, double lidar_f, double lidar_l)
{
  unsigned char flag = 0;
 
    if(lidar_r < warn_range) setbit(flag, 2);
    else clrbit(flag, 2);
 
    if(lidar_f < warn_range) setbit(flag, 1);
    else clrbit(flag, 1);

    if(lidar_l < warn_range) setbit(flag, 0);
    else clrbit(flag, 0);
    
  
    
    switch(flag)
    {
      case 0x01: //             right  font left
                   // x x x x  x  0    0    0// left warn,turn right
        publishTwistCmd(0, -default_yaw_rate);
        break;
        
      case 0x02: // front warn, left and right ok, compare left and right value to turn
        /*if(lidar_l > lidar_r)  */publishTwistCmd(0, default_yaw_rate);
        //else  publishTwistCmd(0, -default_yaw_rate);
        break;
      
      case 0x04: // right warn, turn left
        publishTwistCmd(0, default_yaw_rate); 
        break;
        
      case 0x07: // left,front,right all warn, turn back
        publishTwistCmd(0, 10*default_yaw_rate);
        break;
        
      case 0x03: // left warn, front warn, right ok, turn right
        publishTwistCmd(0, (-default_yaw_rate*2));
        break;
        
      case 0x06: // left ok, front warn, right warn, turn left
        publishTwistCmd(0, (default_yaw_rate*2));
        break;
      
      case 0x05: // left and right warn, front ok, speed up
        publishTwistCmd(2*default_linear_x, 0);
        break;
        
      default: // no warning, ues the des_vel
        publishTwistCmd(des_cmd_array[0], des_cmd_array[1]);
        break;     
    }
    
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "rosmyrobot_node");

  ros::NodeHandle n;

  ros::Rate loop_rate(default_period_hz);

  ros::Subscriber sub_laser = n.subscribe("laser_1", 100, laser_callback);
    
  ros::Subscriber sub_des = n.subscribe("des_vel", 100, des_vel_callback);
  
  twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  
  while (ros::ok())
  {
    rosmyrobot(range_array[0], range_array[1], range_array[2]);
    
    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}


