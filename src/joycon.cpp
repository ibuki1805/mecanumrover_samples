#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


class JoyCtrlMegarover
{
public:
  JoyCtrlMegarover();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_x_, linear_y_, angular_l, angular_r, safety_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


JoyCtrlMegarover::JoyCtrlMegarover():
  linear_x_(1),
  linear_y_(0),
  angular_l(3),
  angular_r(4),
  a_scale_(0.8),
  l_scale_(0.6),
  safety_(7)
{

  nh_.param("/joycon/axis_linear_x", linear_x_, linear_x_);
  nh_.param("/joycon/axis_linear_y", linear_y_, linear_y_);
  nh_.param("/joycon/axis_angular_l", angular_l, angular_l);
  nh_.param("/joycon/axis_angular_r", angular_r, angular_r);
  nh_.param("/joycon/scale_angular", a_scale_, a_scale_);
  nh_.param("/joycon/scale_linear", l_scale_, l_scale_);
  nh_.param("/joycon/safety_button", safety_, safety_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("rover_twist", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &JoyCtrlMegarover::joyCallback, this);

}

void JoyCtrlMegarover::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  if(joy->buttons[safety_]){
    double axes_l = (1-1*joy->axes[angular_l])/2;
    double axes_r = (1+joy->axes[angular_r])/2-1;
    double angular = axes_l+axes_r;
    // std::cout<<"l: "<<axes_l<<" r: "<<axes_r<<" ang: "<<angular<<std::endl;
    twist.angular.z = a_scale_*angular;
    twist.linear.x = l_scale_*joy->axes[linear_x_];
    twist.linear.y = l_scale_*joy->axes[linear_y_];
  }else{
    twist.angular.z =0;
    twist.linear.x = 0;
    twist.linear.y = 0;
  }
  vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joycon");
  JoyCtrlMegarover joy_ctrl_megarover;

  ros::NodeHandle n;

	ros::Rate r(10);
  while(n.ok()){

  	ros::spinOnce();
		r.sleep();
	}
}
