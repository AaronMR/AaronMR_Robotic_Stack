#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"

#include <tf/transform_listener.h>


int aux = 0;
double ranges[362];

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const sensor_msgs::LaserScan& msg)
{
//  ROS_INFO("I heard: [%s]", msg->data.c_str());
  aux = msg.ranges[0];
  ROS_INFO("Yeah= %f",msg.ranges[0] );

  for(unsigned int i = 0; i < 362; ++i){
      ranges[i] = msg.ranges[i] + 1;
  }
  
}


int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher_filter");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan_filtro", 50);

//ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("tilt_scan1", 50);
  ros::Subscriber scan_sub = n.subscribe("scan", 50, chatterCallback);

  
  unsigned int num_readings = 362;
  double laser_frequency = 40;
//  double ranges[num_readings];
  double intensities[num_readings];

  float count = 0.0;
  ros::Rate r(10);

  while(n.ok()){

	ROS_INFO("Antes = %d \n", aux);
	ros::spinOnce();
	ROS_INFO("Despues = %d \n", aux);

    if(count > 5.0)
	count = 0.0;
/*
   //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = count;
      intensities[i] = 100 + count;
    }
*/
    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "base_laser";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 100.0;

    scan.set_ranges_size(num_readings);
    scan.set_intensities_size(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }

    scan_pub.publish(scan);
    count = count + 0.1;
    r.sleep();

  }

}

