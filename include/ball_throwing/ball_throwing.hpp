/*!********************************************************************************
 * \brief     Differential Flatness controller Implementation
 * \authors   Miguel Fernandez-Cortizas
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef __BALL_THROWING_H__
#define __BALL_THROWING_H__

//  ros
#include <cmath>
#include <sys/wait.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "mav_msgs/RateThrust.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"

// Eigen
#include <Eigen/Dense>

// Std libraries
#include <math.h>

#include <array>
#include <iostream>
#include <vector>

// definitions
#define ODOM_TOPIC "odometry"
#define ODOM_TOPIC_TYPE nav_msgs::Odometry
#define SPEED_REFERENCE_TOPIC "motion_reference/speed"
#define SPEED_REFERENCE_TOPIC_TYPE geometry_msgs::TwistStamped
#define POSE_REFERENCE_TOPIC "motion_reference/pose"
#define POSE_REFERENCE_TOPIC_TYPE geometry_msgs::PoseStamped
#define TARGET_POSITION_TOPIC "target_position"

#define RELEASE_BALL_TOPIC "ball/magnet/gain"
#define RELEASE_BALL_TOPIC_TYPE std_msgs::Float32

#define MAX_HEIGHT 4.5
#define LAUNCH_SPEED 4.0
#define BEGIN_POINT_DISTANCE_MARGIN 8
#define THROW_THRESHOLD 0.2
#define Z_OFFSET_BALL 0.4
#define X_OFFSET_BALL 0.0
#define T_DELAY -0.3

using Vector3d = Eigen::Vector3d;

class BallThrowing
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_target_position_;
  ros::Publisher pub_speed_reference_;
  ros::Publisher pub_pose_reference_;
  ros::Publisher pub_release_ball_;

  bool active = false;
  Vector3d marker_position_;
  Vector3d drone_position_;
  Vector3d drone_velocity_;
  Vector3d initial_point_;
  geometry_msgs::TwistStamped speed_to_follow_;

  enum class State
  {
    IDLE,
    WAITING_FOR_REFERENCES,
    APPROACHING_INITIAL_POINT,
    THROWING_TRAJECTORY
  } state_ = State::IDLE;

public:
  BallThrowing()
  {
    sub_odom_ = nh_.subscribe(ODOM_TOPIC, 1, &BallThrowing::CallbackOdomTopic, this);
    sub_target_position_ =
        nh_.subscribe(TARGET_POSITION_TOPIC, 1, &BallThrowing::CallbackTargetPositionTopic, this);
    pub_speed_reference_ = nh_.advertise<geometry_msgs::TwistStamped>(SPEED_REFERENCE_TOPIC, 1);
    pub_pose_reference_ = nh_.advertise<geometry_msgs::PoseStamped>(POSE_REFERENCE_TOPIC, 1);
    pub_release_ball_ = nh_.advertise<std_msgs::Float32>(RELEASE_BALL_TOPIC, 1);
  };

  void computeSpeedToFollow()
  {
    Vector3d speed_to_follow_vector = (marker_position_ - drone_position_);
    speed_to_follow_vector.z() = 0;
    speed_to_follow_vector = speed_to_follow_vector.normalized() * LAUNCH_SPEED;

    speed_to_follow_.twist.linear.x = speed_to_follow_vector(0);
    speed_to_follow_.twist.linear.y = speed_to_follow_vector(1);
    speed_to_follow_.twist.linear.z = speed_to_follow_vector(2);
    speed_to_follow_.header.stamp = ros::Time::now();
  }

  ~BallThrowing(){};
  void run()
  {
    switch (state_)
    {
    case State::IDLE:
      break;
    case State::WAITING_FOR_REFERENCES:
      break;
    case State::APPROACHING_INITIAL_POINT:
    {
      if ((drone_position_ - initial_point_).norm() < 0.2f)
      {
        state_ = State::THROWING_TRAJECTORY;
      }
      else
      {
        pub_pose_reference_.publish(generatePoseMsg(initial_point_));
      }
    }
    break;
    case State::THROWING_TRAJECTORY:
    {
      if (computeBallRelease())
      {
        // pub_pose_reference_.publish(generatePoseMsg(drone_position_ + Vector3d(-1.0, 0.0, 0.1)));
        pub_pose_reference_.publish(generatePoseMsg(Vector3d(5, 0, MAX_HEIGHT)));
        releaseBall();
        state_ = State::IDLE;
        // pub_pose_reference_.publish(generatePoseMsg(drone_position_ + Vector3d(-1.0, 0.0, 0.1)));
      }
      else
      {
        computeSpeedToFollow();
        pub_speed_reference_.publish(speed_to_follow_);
      }
    }
    break;
    default:
      break;
    }
  };

  bool computeBallRelease()
  {
    if (drone_position_.x() > 10.0 || fabs(drone_position_.y()) > 5.5)
    {
      // LOG
      ROS_INFO("Ball released because the drone is too far");
      return true;
    }

    bool release = false;
    Vector3d marker_position_diff = marker_position_ - drone_position_;
    double z_diff = -marker_position_diff.z();
    double z_speed = drone_velocity_.z();
    // double z_speed = 0.0f;
    double t_contact_z = T_DELAY + (z_speed + std::sqrt(z_speed * z_speed + 2 * 9.81f * z_diff)) / 9.81f;

    // check it t_contact_z is nan
    if (std::isnan(t_contact_z))
    {
      return false;
    }
    Vector3d marker_position_contact = drone_position_ + drone_velocity_ * t_contact_z;
    marker_position_contact.z() = marker_position_.z();

    double distance_to_marker = (marker_position_contact - marker_position_).norm();
    // log
    if (distance_to_marker < THROW_THRESHOLD)
    {
      release = true;
    }
    return release;
  }

  void computeInitialPoint()
  {
    // initial_point_ = marker_position_ - Vector3d(1, 0, 0) * BEGIN_POINT_DISTANCE_MARGIN;
    initial_point_ = drone_position_;
    initial_point_.z() = MAX_HEIGHT;
    // LOG
    std::cout << "Initial point: " << initial_point_.transpose() << std::endl;
  }

  void releaseBall()
  {
    std_msgs::Float32 msg;
    msg.data = 0.0f;
    pub_release_ball_.publish(msg);
  }

  geometry_msgs::PoseStamped generatePoseMsg(
      const Eigen::Vector3d &position,
      const Eigen::Quaterniond &orientation = Eigen::Quaterniond::Identity())
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = position.x();
    pose_msg.pose.position.y = position.y();
    pose_msg.pose.position.z = position.z();
    pose_msg.pose.orientation.x = orientation.x();
    pose_msg.pose.orientation.y = orientation.y();
    pose_msg.pose.orientation.z = orientation.z();
    pose_msg.pose.orientation.w = orientation.w();
    return pose_msg;
  }

private:
  void CallbackOdomTopic(const nav_msgs::Odometry &odom_msg)
  {
    if (state_ == State::IDLE)
    {
      state_ = State::WAITING_FOR_REFERENCES;
      // log
      std::cout << "Ball throwing: waiting for references" << std::endl;
    }
    drone_position_ = Vector3d(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y,
                               odom_msg.pose.pose.position.z);
    drone_velocity_ = Vector3d(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y,
                               odom_msg.twist.twist.linear.z);
  };

  void CallbackTargetPositionTopic(const geometry_msgs::PoseStamped &target_position_msg)
  {
    // log
    if (state_ == State::WAITING_FOR_REFERENCES)
    {
      // log
      std::cout << "Ball throwing: received target position" << std::endl;
      state_ = State::APPROACHING_INITIAL_POINT;
      marker_position_ =
          Vector3d(target_position_msg.pose.position.x + X_OFFSET_BALL, target_position_msg.pose.position.y,
                   target_position_msg.pose.position.z + Z_OFFSET_BALL);
      computeInitialPoint();
      pub_pose_reference_.publish(generatePoseMsg(initial_point_));
    }
  };
};

#endif
