/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class R1_teleop_joy
{
public:
  R1_teleop_joy();

private:
  void joyCallback(const joy::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, deadman_axis_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  ros::Timer timer_;
  
};

R1_teleop_joy::R1_teleop_joy():
  ph_("~"),
  linear_(1),
  angular_(2),
  deadman_axis_(4),
  l_scale_(1.0),
  a_scale_(1.0)
{


  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_ = nh_.subscribe<joy::Joy>("joy", 10, &R1_teleop_joy::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&R1_teleop_joy::publish, this));
}

void R1_teleop_joy::joyCallback(const joy::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
 
  vel.linear.x = l_scale_ * joy->axes[1];
  vel.linear.y = a_scale_ * joy->axes[2];
  vel.angular.z = a_scale_ * joy->axes[0];


  vel.angular.x = a_scale_*joy->axes[2];
  vel.angular.y = a_scale_*joy->axes[3];
  //vel.angular.z = a_scale_*joy->axes[2];

  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];

}

void R1_teleop_joy::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);  
  if (deadman_pressed_)
  {
      vel_pub_.publish(last_published_);
  }else{
      last_published_.linear.x = 0.0;
      last_published_.linear.y = 0.0;
      last_published_.angular.z = 0.0;

      last_published_.angular.x = 0.0;
      last_published_.angular.y = 0.0;
      
      
      vel_pub_.publish(last_published_);
  }

  
}

//test
int main(int argc, char** argv)
{
  ros::init(argc, argv, "R1_teleop_joy");
  R1_teleop_joy teleop;

  ros::spin();
}
