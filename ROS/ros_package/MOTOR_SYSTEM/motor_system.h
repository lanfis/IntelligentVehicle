/*
motor <--> system <--> roscore
motor <--> system
  motor_cmd
  motor_status
  activation
  
system <--> roscore
  motion_command
  velocity
  rotate
  
MOTION COMMAND using euler method
[r, theta]
r = velocity, theta = rotate
EXTRA COMMAND
SHUTDOWN
STATUS
*/
/*
TODO
Async communication //Now every command using 50 ms delay, setting power using 150 ms delay
*/
#pragma once
#ifndef _MOTOR_H_
#define _MOTOR_H_


#define delayTime 50000//ns
#define ros_rate 100//Hz

#define ACTIVATE "A"
#define VALUE_GET "G"
#define VALUE_SET "S"
#define POSITIVE "P"
#define NEGATIVE "N"
#define SHUTDOWN "SHUTDOWN"
#define STATUS "STATUS"

#define DEFAULT_SPEED_ACC 10
#define DEFAULT_SPEED_SLD 10
#define DEFAULT_SPEED_RES 10
#define DEFAULT_SPEED_MAX 100


#include <ros/ros.h>
#include <ros/spinner.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <string>
#include <cstring>
#include <sstream>



using namespace ros;
using namespace std;



class Motor_System
{
  public:
    string nodeName;
    //motor
    string motor_system_motor_status_sub;
    string motor_system_motor_command_pub;
    //computer
    string motor_system_motion_velocity_pub;
    string motor_system_motion_rotate_pub;
    //const string motor_system_motor_rpm_pub = nodeName+"/motor_rpm_";
    string motor_system_motion_command_sub;
    string motor_system_activation_sub;

  private:
    float ver_;
    string motion_command_;
    
    string motor_status_;
    int motor_rpm_size_ = 2;
    int* motor_rpm_;
    
  private:
    ros::AsyncSpinner spinner;
    ros::NodeHandle n_;
    ros::Subscriber motor_system_motor_status_sub_;
    ros::Publisher motor_system_motor_command_pub_;
    //ros::Publisher motor_system_motor_rpm_pub_;
    
    ros::Publisher motor_system_motion_velocity_pub_;
    ros::Publisher motor_system_motion_rotate_pub_;
    ros::Subscriber motor_system_motion_command_sub_;
    ros::Subscriber motor_system_activation_sub_;

    std_msgs::String msg_motor_cmd_;
    std_msgs::Float32 msg_velocity_;
    std_msgs::Float32 msg_rotate_;

    void info_show();
    void ros_param();
    void ros_param_set();
    void ros_param_get();
    
    
    void status_show_(int& motor_rpm);
    void motion_function(int* motor_n, int* motor_o, int& motor_size);
    
    void motor_cmd_value_set(int* motor, int& motor_size);
    void motor_cmd_activation();
    void motor_cmd_value_get();
    void motor_status_callBack(const std_msgs::String::ConstPtr& msg);
    void motion_command_callBack(const std_msgs::String::ConstPtr& msg);
    void activation_callBack(const std_msgs::Bool::ConstPtr& msg);

  public:   
    bool flag_act;
    bool flag_status_show;
    
    int speed_acc;
    int speed_sld;
    int speed_res;
    int speed_max;
    
    float motion_velocity;
    float motion_rotate;
    
    float velocity;
    float rotate;

  public:
    Motor_System(int thread);
    ~Motor_System();
    void sub_init();
    void pub_init();
    void run();
    void status_show(int* motor_rpm_, int& motor_rpm_size_);
    void shutdown(void);
};

Motor_System::Motor_System(int thread) : spinner(thread)
{ 
  ver_ = 1.0;
  nodeName = "Motor_System";
  //motor
  motor_system_motor_status_sub = nodeName+"/motor_status";
  motor_system_motor_command_pub = nodeName+"/motor_cmd";
  //computer
  motor_system_motion_velocity_pub = nodeName+"/velocity";
  motor_system_motion_rotate_pub = nodeName+"/rotate";
  //const string motor_system_motor_rpm_pub = nodeName+"/motor_rpm_";
  motor_system_motion_command_sub = nodeName+"/motion_command";
  motor_system_activation_sub = nodeName+"/activation";

  flag_act = true;
  flag_status_show, true;
  speed_acc = DEFAULT_SPEED_ACC;
  speed_sld = DEFAULT_SPEED_SLD;
  speed_res = DEFAULT_SPEED_RES;
  speed_max = DEFAULT_SPEED_MAX;
  ros_param();
  ros_param_get();
  
  motor_rpm_ = new int [motor_rpm_size_];
  for(int i = 0; i < motor_rpm_size_; i++)
      motor_rpm_[i] = 0;

  motion_velocity = 0;
  motion_rotate = 0;
  velocity = 0;
  rotate = 0;

//  motor_cmd_value_get();
//  info_show();
  pub_init();
  sub_init();
  spinner.start();
  run();
}

Motor_System::~Motor_System()
{
    delete [] motor_rpm_;
}

void Motor_System::shutdown(void)
{
  while(motor_rpm_[0] != 0 || motor_rpm_[1] != 0)
  {
    for(int i = 0; i < 2; i++)
    { 
      if(motor_rpm_[i] == 0)
        continue;
      motor_rpm_[i] = (motor_rpm_[i] > 0)? (motor_rpm_[i] - 1) : (motor_rpm_[i] + 1);
    }
    motor_cmd_value_set(motor_rpm_, motor_rpm_size_);
    motor_cmd_activation();
    motor_cmd_value_get();
    velocity = float(motor_rpm_[0] + motor_rpm_[1]) / 2.0;
    rotate = float(motor_rpm_[0] - motor_rpm_[1]);
    info_show();
  }
}

void Motor_System::motor_status_callBack(const std_msgs::String::ConstPtr& msg)
{ 
  motor_status_ = string(msg -> data);
  
  string str;
  int motor_idx;
  int motor_power;
  stringstream token(motor_status_);
  for(int i = 0; i < motor_rpm_size_; i++)
  {
    token >> str;
    stringstream tmp;
    tmp << str.substr(1, str.length()-1);
    tmp >> motor_idx;
    tmp.clear();
    motor_power = motor_idx % 1000;
    motor_idx = motor_idx / 1000;
    str = str[0];
    if(str == POSITIVE)
      motor_rpm_[motor_idx - 1] = motor_power;
    if(str == NEGATIVE)
      motor_rpm_[motor_idx - 1] = -1 * motor_power;
  }
  velocity = float(motor_rpm_[0] + motor_rpm_[1]) / 2.0;
  rotate = float(motor_rpm_[0] - motor_rpm_[1]);

  info_show();
}


void Motor_System::motion_command_callBack(const std_msgs::String::ConstPtr& msg)
{ 
  if(!flag_act)
      return;
  ros_param_get();
  
  motion_command_ = string(msg -> data);
  if(motion_command_ == SHUTDOWN)
  {
    shutdown();
    return;
  }
  if(motion_command_ == STATUS)
  {
    motor_cmd_value_get();
    return;
  }
  stringstream token(motion_command_);

  token >> motion_velocity; 
  token >> motion_rotate;
  
  int *motor_ = new int [motor_rpm_size_];
  motor_[0] = int(motion_velocity * 2 + motion_rotate) / 2;
  motor_[1] = int(motion_velocity * 2 - motion_rotate) / 2;
  
  motion_function(motor_, motor_rpm_, motor_rpm_size_);
  delete [] motor_;
  
  motor_cmd_activation();
  motor_cmd_value_get();
}

void Motor_System::activation_callBack(const std_msgs::Bool::ConstPtr& msg)
{
    flag_act = msg -> data;
}

void Motor_System::motion_function(int* motor_n, int* motor_o, int& motor_size)
{
  int *d = new int [motor_size];
  for(int i = 0; i < motor_size; i++)
  {
    if(motor_n[i] > motor_o[i])
    {
      d[i] = (((motor_n[i] - motor_o[i]) / speed_res) > speed_acc)? speed_acc : ((motor_n[i] - motor_o[i]) / speed_res);
      d[i] = (d[i] > 0)? d[i] : 1;
      motor_n[i] = (motor_o[i] + d[i] > speed_max)? (speed_max) : (motor_o[i] + d[i]);
    }
    else if(motor_n[i] < motor_o[i])
    {
      d[i] = ((-1*(motor_n[i] - motor_o[i]) / speed_res) > speed_sld)? speed_sld : (-1*(motor_n[i] - motor_o[i]) / speed_res);
      d[i] = (d[i] > 0)? d[i] : 1;
      motor_n[i] = (motor_o[i] - d[i] < -1*speed_max)? (-1*speed_max) : (motor_o[i] - d[i]);
    }
  }
  motor_cmd_value_set(motor_n, motor_size);
  
  delete [] d;
}

void Motor_System::motor_cmd_value_set(int* motor, int& motor_size)
{
  string str = VALUE_SET; str += " ";
  string tmp;
  int num;
  stringstream token;
  for(int i = 0; i < motor_size; i++)
  {
    if(motor[i] >=0)
      str += POSITIVE;
    else 
      str += NEGATIVE;
    num = (i+1) * 1000 + abs(motor[i]);
    token << num;
    token >> tmp;
    token.clear();
    str += tmp;
    str += " ";
  }
  msg_motor_cmd_.data = str.c_str();
  motor_system_motor_command_pub_.publish(msg_motor_cmd_);
  usleep(delayTime);
}

void Motor_System::motor_cmd_activation()
{
  string str = ACTIVATE;
  msg_motor_cmd_.data = str.c_str();
  motor_system_motor_command_pub_.publish(msg_motor_cmd_);
  usleep(delayTime);
}

void Motor_System::motor_cmd_value_get()
{
  string str = VALUE_GET;
  msg_motor_cmd_.data = str.c_str();
  motor_system_motor_command_pub_.publish(msg_motor_cmd_);
  usleep(delayTime);
}


void Motor_System::status_show(int* motor_rpm_, int& motor_rpm_size_)
{  
  if(!flag_status_show)
      return;
  for(int i = 0; i < motor_rpm_size_; i++)
  {
    cout << "Motor " << i+1 << " : ";
    status_show_(motor_rpm_[i]);
    cout << float(motor_rpm_[i]) / float(DEFAULT_SPEED_MAX) * 100.0 << " %";
    cout << " ";
  }
    
  cout << " Velocity : " << float(velocity) << " / " << "Rotate : " << float(rotate) <<"     \r";
  cout.flush();
}

void Motor_System::status_show_(int& motor_rpm)
{
    int barWidth = 21;
    int center = (barWidth + 1) / 2;
    int value = int(float(motor_rpm) / float(barWidth-1) * 2.0);
    int barHeader = (value > 0)? center : (center + value);
    int barTailer = (value > 0)? (center + value) : center;
    cout << "[" ;
    for(int i = 0; i < barWidth; i++)
    {
        if(i == center)
            cout << "|";
        else if(barHeader <= i && i <= barTailer)
            cout << "+";
        else
            cout << " ";
    }
    cout << "]";
    return;
}



void Motor_System::run()
{
  ros_param_get();
  motor_cmd_value_get();
  info_show();
}

void Motor_System::pub_init()
{
  motor_system_motor_command_pub_ = n_.advertise< std_msgs::String>(motor_system_motor_command_pub.c_str(), 10);
  //motor_system_motor_rpm_pub_ = n_.advertise< std_msgs::String>(motor_system_motor_rpm_pub.c_str(), 1);
  motor_system_motion_velocity_pub_ = n_.advertise< std_msgs::Float32>(motor_system_motion_velocity_pub.c_str(), 1);
  motor_system_motion_rotate_pub_ = n_.advertise< std_msgs::Float32>(motor_system_motion_rotate_pub.c_str(), 1);
}

void Motor_System::sub_init()
{
  motor_system_motor_status_sub_ = n_.subscribe(motor_system_motor_status_sub.c_str(), 10, &Motor_System::motor_status_callBack, this);
  motor_system_motion_command_sub_ = n_.subscribe(motor_system_motion_command_sub.c_str(), 10, &Motor_System::motion_command_callBack, this);
  motor_system_activation_sub_ = n_.subscribe(motor_system_activation_sub.c_str(), 1, &Motor_System::activation_callBack, this);
}

void Motor_System::info_show()
{
  msg_velocity_.data = velocity;
  msg_rotate_.data = rotate;
  motor_system_motion_velocity_pub_.publish(msg_velocity_);
  motor_system_motion_rotate_pub_.publish(msg_rotate_);
  status_show(motor_rpm_, motor_rpm_size_);
}

void Motor_System::ros_param()
{
  n_.param(nodeName+"/ver", ver_, ver_); 
//  n_.param(nodeName+"/act", flag_act, true); 
  n_.param(nodeName+"/status_show", flag_status_show, true); 
  n_.param(nodeName+"/speed_acc", speed_acc, DEFAULT_SPEED_ACC);
  n_.param(nodeName+"/speed_sld", speed_sld, DEFAULT_SPEED_SLD);
  n_.param(nodeName+"/speed_res", speed_res, DEFAULT_SPEED_RES);
  n_.param(nodeName+"/speed_max", speed_max, DEFAULT_SPEED_MAX);
}
  
void Motor_System::ros_param_get()
{
//  n_.getParam(nodeName+"/act", flag_act);
  n_.getParam(nodeName+"/status_show", flag_status_show);
  n_.getParam(nodeName+"/speed_acc", speed_acc);
  n_.getParam(nodeName+"/speed_sld", speed_sld);
  n_.getParam(nodeName+"/speed_res", speed_res);
  n_.getParam(nodeName+"/speed_max", speed_max);
}

void Motor_System::ros_param_set()
{
//  n_.setParam(nodeName+"/act", flag_act);
  n_.setParam(nodeName+"/status_show", flag_status_show);
  n_.setParam(nodeName+"/speed_acc", speed_acc);
  n_.setParam(nodeName+"/speed_sld", speed_sld);
  n_.setParam(nodeName+"/speed_res", speed_res);
  n_.setParam(nodeName+"/speed_max", speed_max);
}

#endif