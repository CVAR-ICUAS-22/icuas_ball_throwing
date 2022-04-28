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

#define RELEASE_BALL_TOPIC "uav_magnet/gain"
#define RELEASE_BALL_TOPIC_TYPE std_msgs::Float32

#define MAX_HEIGHT 4.5
#define LAUNCH_SPEED 4.0
#define BEGIN_POINT_DISTANCE_MARGIN 8
#define BEGIN_POINT_HEIGHT 1.0
#define THROW_THRESHOLD 0.2
#define Z_OFFSET_BALL 0.4
#define X_OFFSET_BALL 0.0
#define T_DELAY 0.0 // 0.3

#define MAX_X 11.5
#define MAX_Y 6.5

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

  float launch_distance_;
  float launch_height_;
  bool ball_released_ = false;
  Vector3d launch_trajectory_endpoint_;
  Vector3d tag_orientation_;

  enum class State
  {
    IDLE,
    WAITING_FOR_REFERENCES,
    APPROACHING_INITIAL_POINT,
    THROWING_TRAJECTORY
  } state_ = State::IDLE;

public:
  BallThrowing();
  ~BallThrowing(){};

  void run();

  void computeSpeedToFollow();
  bool computeBallRelease();
  void computeInitialPoint();
  void releaseBall();
  void computeLaunchingParameters();

  geometry_msgs::PoseStamped generatePoseMsg(
      const Eigen::Vector3d &position,
      const Eigen::Quaterniond &orientation = Eigen::Quaterniond::Identity());

private:
  void CallbackOdomTopic(const nav_msgs::Odometry &odom_msg);
  void CallbackTargetPositionTopic(const geometry_msgs::PoseStamped &target_position_msg);
};

Vector3d identifyTagOrientation(const Vector3d tag_position_);

#endif
