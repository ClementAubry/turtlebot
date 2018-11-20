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
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

#include <limits.h>
#include <string.h>
#include <sstream>
#include <iostream>

class TurtlebotTeleop
{
  public:
    TurtlebotTeleop();

  private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void publish();

    ros::NodeHandle ph_,
        nh_;

    int linear_, angular_, deadman_axis_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

    geometry_msgs::Twist last_published_;
    boost::mutex publish_mutex_;
    bool deadman_pressed_;
    bool zero_twist_published_;
    ros::Timer timer_;
    bool calibrating_;
    bool calibrating_zero_;
    bool calibrated_;

    // min and max values of axes for calibration
    double l_min_, l_max_;
    double a_min_, a_max_;
    double a_offset_, l_offset_;
};

TurtlebotTeleop::TurtlebotTeleop() : ph_("~"),
                                     linear_(1),
                                     angular_(0),
                                     deadman_axis_(4),
                                     calibrating_(false),
                                     calibrating_zero_(false),
                                     calibrated_(true),
                                     l_scale_(0.3),
                                     a_scale_(0.9),
                                     l_offset_(std::numeric_limits<float>::max()),
                                     a_offset_(std::numeric_limits<float>::max()),
                                     l_min_(std::numeric_limits<float>::max()),
                                     a_min_(std::numeric_limits<float>::max()),
                                     l_max_(-std::numeric_limits<float>::max()),
                                     a_max_(-std::numeric_limits<float>::max())
{
    ph_.param("axis_linear", linear_, linear_);
    ph_.param("axis_angular", angular_, angular_);
    ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
    ph_.param("scale_angular", a_scale_, a_scale_);
    ph_.param("scale_linear", l_scale_, l_scale_);
    ph_.param("scale_linear", l_scale_, l_scale_);
    ph_.param("is_calibrated", calibrated_, true);
    ph_.param("axis_lin_min",l_min_,-1.0);
    ph_.param("axis_lin_max",l_max_, 1.0);
    ph_.param("axis_lin_mean",l_offset_, 0.5);
    ph_.param("axis_ang_mean", a_offset_, 0.5);
    ph_.param("axis_ang_min",a_min_ , -1.0);
    ph_.param("axis_ang_max", a_max_, 1.0);

    if (!calibrated_)
        ROS_INFO("Please calibrate your Genius Joystick pressing (and holding) buttons 7,8,9");

    deadman_pressed_ = false;
    zero_twist_published_ = false;

    vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TurtlebotTeleop::joyCallback, this);

    timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&TurtlebotTeleop::publish, this));
}

double remap(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void TurtlebotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    geometry_msgs::Twist vel;
    ROS_INFO("Received lin=[%f],ang=[%f]: ", joy->axes[linear_], joy->axes[angular_]);
    // ROS_INFO("Comparing with min/max x=[%f/%f],z=[%f/%f]: ", l_min_, l_max_, a_min_, a_max_);

    if (joy->buttons[7] && joy->buttons[8] && joy->buttons[9] && !calibrated_)
    {
        ROS_INFO("Now move the stick forward, backward, right and left several times. When OK, release middle button");
        ROS_INFO("When OK, release the stick AND THEN, release middle button");
        if (joy->axes[linear_] > l_max_)
        {
            l_max_ = joy->axes[linear_];
        }
        if (joy->axes[linear_] < l_min_)
        {
            l_min_ = joy->axes[linear_];
        }
        if (joy->axes[angular_] > a_max_)
        {
            a_max_ = joy->axes[angular_];
        }
        if (joy->axes[angular_] < a_min_)
        {
            a_min_ = joy->axes[angular_];
        }
    }
    else if (joy->buttons[7] && !joy->buttons[8] && joy->buttons[9] && !calibrated_)
    {
        ROS_INFO("Please don't move the stick and release");
        a_offset_ = joy->axes[angular_];
        l_offset_ = joy->axes[linear_];
        calibrated_ = true;
    }
    if (calibrated_)
    {
        // ROS_INFO("Valeur angular : [%f]", joy->axes[angular_]);
        //Si on est dans la bande des +- 12,5%de la variation max (25% centré en zéro), on envoi zéro.
        if (!joy->axes[angular_] < a_offset_ + 0.125 * (a_max_ - a_min_) || !joy->axes[angular_] > a_offset_ - 0.125 * (a_max_ - a_min_))
        {
            // ROS_INFO("Ca bouge");
            if (joy->axes[angular_] >= a_offset_ + 0.125 * (a_max_ - a_min_))
                vel.angular.z = remap(joy->axes[angular_], a_offset_ + 0.125 * (a_max_ - a_min_), a_max_, 0, a_scale_);
            if (joy->axes[angular_] <= a_offset_ - 0.125 * (a_max_ - a_min_))
                vel.angular.z = remap(joy->axes[angular_], a_min_, a_offset_ - 0.125 * (a_max_ - a_min_), -a_scale_, 0);
        }
        // ROS_INFO("Valeur linear : [%f]", joy->axes[linear_]);
        // ROS_INFO("l_offset_ + 0.125 * (l_max_ - l_min_)=[%f]", l_offset_ + 0.125 * (l_max_ - l_min_));
        // ROS_INFO("l_offset_ - 0.125 * (l_max_ - l_min_)=[%f]", l_offset_ - 0.125 * (l_max_ - l_min_));
        if (!joy->axes[linear_] < l_offset_ + 0.125 * (l_max_ - l_min_) || !joy->axes[linear_] > l_offset_ - 0.125 * (l_max_ - l_min_))
        {
            // ROS_INFO("Ca bouge");
            if (joy->axes[linear_] >= l_offset_ + 0.125 * (l_max_ - l_min_))
                vel.linear.x = remap(joy->axes[linear_], l_offset_ + 0.125 * (l_max_ - l_min_), l_max_, 0, l_scale_);
            if (joy->axes[linear_] <= l_offset_ - 0.125 * (l_max_ - l_min_))
                vel.linear.x = remap(joy->axes[linear_], l_min_, l_offset_ - 0.125 * (l_max_ - l_min_), -l_scale_, 0);
        }

        // vel.angular.z = a_scale_ * remap(joy->axes[angular_], a_min_, a_max_);
        // ROS_INFO("0.30 * a_scale_ = [%f]", 0.30 * a_scale_);
        // ROS_INFO("fabs(vel.angular.z) = [%f]", fabs(vel.angular.z));
        // vel.linear.x = l_scale_ * remap(joy->axes[linear_], l_min_, l_max_,-l_scale_,l_scale_);
        // ROS_INFO("0.30 * l_scale_ = [%f]", 0.30 * l_scale_);
        // ROS_INFO("fabs(vel.linear.x) = [%f]", fabs(vel.linear.x));
        last_published_ = vel;
        ROS_INFO("After remap lin=[%f],ang=[%f]: ", vel.linear.x, vel.angular.z);
    }
    deadman_pressed_ = joy->buttons[deadman_axis_] && calibrated_;
    // ROS_INFO("min/max lin=[%f/%f],ang=[%f/%f]: ", l_min_, l_max_, a_min_, a_max_);
    // ROS_INFO("offset lin=[%f],ang=[%f]\n\n", l_offset_, a_offset_);
    // ROS_INFO("offset remaped lin=[%f],ang=[%f]\n\n", l_scale_ * remap(l_offset_, l_min_, l_max_, -l_scale_, l_scale_), a_scale_ * remap(a_offset_, a_min_, a_max_, -a_scale_, a_scale_));
}

void TurtlebotTeleop::publish()
{
    boost::mutex::scoped_lock lock(publish_mutex_);

    if (deadman_pressed_)
    {
        vel_pub_.publish(last_published_);
        zero_twist_published_ = false;
    }
    else if (!deadman_pressed_ && !zero_twist_published_)
    {
        vel_pub_.publish(*new geometry_msgs::Twist());
        zero_twist_published_ = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_teleop");
    TurtlebotTeleop turtlebot_teleop;

    ros::spin();
}
