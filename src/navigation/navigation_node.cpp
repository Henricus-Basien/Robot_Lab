#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <geometry_msgs/Twist.h>
#include "navigation/MotorCommand.h"


class Navigator
{
public:
  Navigator();
  void navLoop();
  void keyboardCallback(geometry_msgs::Twist);

private:
  ros::NodeHandle nh_;
  ros::Subscriber keySub;
  //ros::Publisher nav_pub_;

  int gain_range_;
  double max_vel_;

  int motor_gain_L_,motor_gain_R_;
  
};

void Navigator::keyboardCallback(geometry_msgs::Twist vel)
  {
    puts("-------------------------");
    ROS_INFO("Input:\n   Linear (x): %f\n   Angular (z): %f\n",vel.linear.x,vel.angular.z);

    motor_gain_L_= (int) ((vel.linear.x-vel.angular.z)/2*gain_range_);
    motor_gain_R_= (int) ((vel.linear.x+vel.angular.z)/2*gain_range_);

    ROS_INFO("Output:\n   Motor Gain Left (x): %d\n   Motor Gain Right: %d\n",motor_gain_L_,motor_gain_R_);
    /*
    ::messages::MotorCommand motorCommand;

    motorCommand.gain1 = motor_gain_L_;
    motorCommand.gain2 = motor_gain_R_;

    nav_pub_.publish(Gains);   
    */

  }


Navigator::Navigator():
	gain_range_(255),
  max_vel_(0.8),
	motor_gain_L_(0),
	motor_gain_R_(0)
{

  keySub = nh_.subscribe("key_input", 1000, &Navigator::keyboardCallback,this);
  //nav_pub_ = nh_.advertise<::messages::MotorCommand>("Motor Output",1000);

  ROS_INFO_STREAM("Navigator Initialized.");
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigator");
  Navigator navigator;

  //signal(SIGINT,quit);

  navigator.navLoop();
  
  return(0);
}


void Navigator::navLoop()
{



  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Navigation is waiting for input...");

  ros::spin();
  /*
  for(;;)
  {

  }
  */


  return;
}