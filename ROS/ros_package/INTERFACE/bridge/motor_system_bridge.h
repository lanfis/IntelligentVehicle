#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

using namespace ros;
using namespace std;


class Motor_System_Bridge
{
  private:
    float ver_;
    ros::NodeHandle n_;
    ros::Publisher  motor_activation_pub;
    ros::Publisher  motor_command_pub;
    ros::Subscriber motor_velocity_sub;
    ros::Subscriber motor_rotate_sub;
    
    int queue_size;
    std_msgs::Bool    msg_activation_;
    std_msgs::String  msg_motor_cmd_;
    std_msgs::Float32 msg_velocity_;
    std_msgs::Float32 msg_rotate_;
    
  private:
    void pub_init();
    void sub_init();
    void motor_velocity_callBack(const std_msgs::Float32::ConstPtr& msg);
    void motor_rotate_callBack(const std_msgs::Float32::ConstPtr& msg);

  public:
    string topic_motor_activation_pub;
    string topic_motor_command_pub;
    string topic_motor_velocity_sub;
    string topic_motor_rotate_sub;
    bool activation;
    bool update_command;
    string motor_command;
    bool flag_update_velocity;
    float velocity;
    bool flag_update_rotate;
    float rotate;
    
  public:
    Motor_System_Bridge(ros::NodeHandle& n);
    ~Motor_System_Bridge();
    void run();
};

Motor_System_Bridge::Motor_System_Bridge(ros::NodeHandle& n) : n_(n)
{
  ver_ = 1.0;
  topic_motor_activation_pub = "motor_system/activation";
  topic_motor_command_pub = "motor_system/command";
  topic_motor_velocity_sub = "motor_system/velocity";;
  topic_motor_rotate_sub = "motor_system/rotate";;
  queue_size = 4;

  activation = true;
  update_command = false;
  flag_update_velocity = false;
  flag_update_rotate = false;
  pub_init();
  sub_init();
}

void Motor_System_Bridge::pub_init()
{
  motor_activation_pub = n_.advertise< std_msgs::Bool>(topic_motor_activation_pub.c_str(), 1);
  motor_command_pub = n_.advertise< std_msgs::String>(topic_motor_command_pub.c_str(), queue_size);
}

void Motor_System_Bridge::sub_init()
{
  motor_velocity_sub = n_.subscribe(topic_motor_velocity_sub.c_str(), queue_size, &Motor_System_Bridge::motor_velocity_callBack, this);
  motor_rotate_sub = n_.subscribe(topic_motor_rotate_sub.c_str(), queue_size, &Motor_System_Bridge::motor_rotate_callBack, this);
}

Motor_System_Bridge::~Motor_System_Bridge()
{
}

void Motor_System_Bridge::run()
{
  if(update_command)
  {
    msg_activation_.data = activation;
    msg_motor_cmd_.data = motor_command.c_str();
    motor_activation_pub.publish(msg_activation_);
    motor_command_pub.publish(msg_motor_cmd_);
    update_command = false;
  }
}
  
void Motor_System_Bridge::motor_velocity_callBack(const std_msgs::Float32::ConstPtr& msg)
{ 
  if(!activation) return;
  //msg_velocity_ = msg;
  velocity = msg -> data;
  flag_update_velocity = true;
}

void Motor_System_Bridge::motor_rotate_callBack(const std_msgs::Float32::ConstPtr& msg)
{ 
  if(!activation) return;
  //msg_rotate_ = msg;
  rotate = msg -> data;
  flag_update_rotate = true;
}

