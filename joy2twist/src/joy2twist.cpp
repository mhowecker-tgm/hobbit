//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <string.h>
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class PS3Joystick2Twist
{
  public: 
    PS3Joystick2Twist();
    void SetFastSpeed(void);

  private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    int linear_axis_, angular_axis_;
    double linear_, angular_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
PS3Joystick2Twist::PS3Joystick2Twist() : priv_nh_("~")
{
  linear_ = 0.2; //0.15;
  angular_ = 0.41888; //0.31416;
  linear_axis_ = 1;
  angular_axis_ = 0;

  priv_nh_.param("linear_axis", linear_axis_, linear_axis_);
  priv_nh_.param("angular_axis", angular_axis_, angular_axis_);
  priv_nh_.param("linear_speed_max", linear_, linear_);
  priv_nh_.param("angular_speed_max", angular_, angular_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &PS3Joystick2Twist::joyCallback, this);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void PS3Joystick2Twist::SetFastSpeed(void)
{
  linear_ = 0.4;
  angular_ = 0.6283;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void PS3Joystick2Twist::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(joy->buttons[4] > 0)
  {
      geometry_msgs::Twist twist;
      twist.linear.x = linear_ * joy->axes[linear_axis_];
      twist.angular.z = angular_ * joy->axes[angular_axis_];
      vel_pub_.publish(twist);
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char * argv[])
{
  ros::init(argc, argv, "ps3joystick2twist");
  PS3Joystick2Twist teleop_harlie;

  if (argc == 2)
  {
    if (strcmp(argv[1], "--fast") == 0)
    {
      printf("Using fast mode!\n");
      teleop_harlie.SetFastSpeed();
    }
  }

  ros::spin();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

