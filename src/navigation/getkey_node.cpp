#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <geometry_msgs/Twist.h>

#define KEY_RIGHT 0x43
#define KEY_LEFT 0x44
#define KEY_UP 0x41
#define KEY_DOWN 0x42
#define KEY_Q 0x71

#define KEY_W 0x77
#define KEY_A 0x61
#define KEY_S 0x73
#define KEY_D 0x64

class KeyFetcher
{
public:
  KeyFetcher();
  void keyLoop();

private:

  ros::NodeHandle nh_;
  double linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  
};

KeyFetcher::KeyFetcher():
  linear_(0),
  angular_(0),
  l_scale_(1.0),
  a_scale_(1.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("key_input",1000);
  ROS_INFO_STREAM("KeyFetcher Intialized.");
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
  ros::init(argc, argv, "keyfetcher");
  KeyFetcher keyfetcher;

  signal(SIGINT,quit);

  keyfetcher.keyLoop();
  
  return(0);
}


void KeyFetcher::keyLoop()
{
  char c;
  bool input=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");

  geometry_msgs::Twist vel;

  for(;;)
  {
    linear_=angular_=0;

    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    ROS_DEBUG("value: 0x%02X\n", c);//printf("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEY_LEFT:
        //ROS_DEBUG("LEFT");
        puts("LEFT");
        angular_ = 1.0;
        input = true;
        break;
      case KEY_RIGHT:
        //ROS_DEBUG("RIGHT");
        puts("RIGHT");
        angular_ = -1.0;
        input = true;
        break;
      case KEY_UP:
        //ROS_DEBUG("UP");q
        puts("UP");
        linear_ = 1.0;
        input = true;
        break;
      case KEY_DOWN:
        //ROS_DEBUG("DOWN");
        puts("DOWN");
        linear_ = -1.0;
        input = true;
        break;
      //---------------------
      case KEY_W:
        //ROS_DEBUG("W");
        puts("W");
        linear_ = 0.5;
        input = true;
        break;
      case KEY_A:
        //ROS_DEBUG("A");
        puts("A");
        angular_ = 0.5;
        input = true;
        break;
      case KEY_S:
        //ROS_DEBUG("S");q
        puts("S");
        linear_ = -0.5;
        input = true;
        break;
      case KEY_D:
        //ROS_DEBUG("D");
        puts("D");
        angular_ = -0.5;
        input = true;
        break;
    }   

    //geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*angular_;
    vel.linear.x = l_scale_*linear_;

    printf("Output:\n   Linear (x): %f\n   Angular (z): %f\n",vel.linear.x,vel.angular.z);

    if(input ==true)
    {
      vel_pub_.publish(vel);    
      input=false;
    }
  }


  return;
}