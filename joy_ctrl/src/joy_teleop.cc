#include "ros/ros.h"
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleOpBox
{
public:
	TeleOpBox();
private:
	void joyCB(const sensor_msgs::Joy::ConstPtr& joy);
	ros::NodeHandle nh_;
	int linear_, angular_x_,angular_y_,angular_z_;
	double l_scale_, a_scale_x, a_scale_y, a_scale_z;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;
};

TeleOpBox::TeleOpBox() : linear_(1), angular_x_(2), angular_y_(3), angular_z_(0)
{
	nh_.param("axis_linear", linear_, linear_);
	nh_.param("axis_angular_z",angular_z_, angular_z_);
	nh_.param("axis_angular_x",angular_x_, angular_x_);
	nh_.param("axis_angular_y",angular_y_, angular_y_);
	nh_.param("scale_angular_x", a_scale_x, a_scale_x);
	nh_.param("scale_angular_y", a_scale_y, a_scale_y);
	nh_.param("scale_angular_z", a_scale_z, a_scale_z);
	nh_.param("scale_linear", l_scale_, l_scale_);

//	vel_pub_ = nh_.advertise<geometry_msgs::Wrench>("multicopter/torque_cmd",1);
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("multicopter/twist_cmd",1);
	
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy",10 , &TeleOpBox::joyCB, this);
}

void TeleOpBox::joyCB(const sensor_msgs::Joy::ConstPtr& joy)
{
//	geometry_msgs::Wrench wrench;
//	wrench.force.z = l_scale_*joy->axes[linear_];
//	wrench.torque.x = -a_scale_*joy->axes[angular_x_];
//	wrench.torque.y = -a_scale_*joy->axes[angular_y_];
//	wrench.torque.z = a_scale_*joy->axes[angular_z_];
//	geometry_msgs::Wrench wrench;
	geometry_msgs::Twist twist;
	twist.linear.z = l_scale_*joy->axes[linear_];
	twist.angular.x = a_scale_x*joy->axes[angular_x_];
	twist.angular.y = -a_scale_y*joy->axes[angular_y_];
	twist.angular.z = -a_scale_z*joy->axes[angular_z_];
	vel_pub_.publish(twist);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_box");
	TeleOpBox teleop_box;
	ros::spin();
}
