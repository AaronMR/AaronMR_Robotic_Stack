#include <ros/ros.h>
#include <joy/Joy.h>
#include <geometry_msgs/Twist.h>

const double max_speed = 1.5;
double vel_x, vel_y, vel_rot;

void joyCallback( const joy::JoyConstPtr& joy_msg )
{
	if( joy_msg->axes.size() < 3 )
	{
		ROS_ERROR( "Too few joystick axes: %d (expected: 3+)", joy_msg->axes.size() );
		return;
	}

	ROS_DEBUG("Received [axis0=%f axis1=%f axis2=%f]", joy_msg->axes[0], joy_msg->axes[1], joy_msg->axes[2] );

	double throttle = 1.0;
	if( joy_msg->axes.size() >= 4 )
	{
		throttle = (joy_msg->axes[3] + 1.0) / 2.0;	// map [-1; 1] --> [0; 1]
	}

	vel_x = joy_msg->axes[1] * throttle * max_speed;
	vel_y = joy_msg->axes[2] * throttle * max_speed;
	vel_rot = joy_msg->axes[0];
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "R1_teleop_joy_node");
	ros::NodeHandle n;

	ros::Publisher  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	ros::Subscriber joy_sub = n.subscribe( "joy", 10, &joyCallback );

	ros::Rate loop_rate(20);
	while( ros::ok() )
	{
		ros::spinOnce();

		geometry_msgs::Twist twist_msg;
		twist_msg.linear.x = vel_x;
		twist_msg.linear.y = vel_y;
		twist_msg.angular.z = vel_rot;

		vel_pub.publish(twist_msg);

		loop_rate.sleep();
	}
}

