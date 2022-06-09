/*!********************************************************************************
 * \brief    Ball throwing
 * \copyright Copyright (c) 2022 Universidad Politecnica de Madrid
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

#include "ball_throwing.hpp"

BallThrowing::BallThrowing()
{
  sub_odom_ = nh_.subscribe(ODOM_TOPIC, 1, &BallThrowing::CallbackOdomTopic, this);
  sub_target_position_ =
      nh_.subscribe(TARGET_POSITION_TOPIC, 1, &BallThrowing::CallbackTargetPositionTopic, this);
  pub_speed_reference_ = nh_.advertise<SPEED_REFERENCE_TOPIC_TYPE>(SPEED_REFERENCE_TOPIC, 1);
  pub_pose_reference_ = nh_.advertise<POSE_REFERENCE_TOPIC_TYPE>(POSE_REFERENCE_TOPIC, 1);
  waypoint_pub_ = nh_.advertise<WAYPOINT_TOPIC_TYPE>(WAYPOINT_TOPIC, 1);
  pub_release_ball_ = nh_.advertise<RELEASE_BALL_TOPIC_TYPE>(RELEASE_BALL_TOPIC, 1);

  computeLaunchingParameters();
}

void BallThrowing::run()
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
      // pub_pose_reference_.publish(generatePoseMsg(initial_point_, setOrientationFromTag(marker_position_)));
      waypoint_pub_.publish(generateWaypointMsg(initial_point_, setOrientationFromTag(marker_position_)));
    }
  }
  break;
  case State::THROWING_TRAJECTORY:
  {
    if (ball_released_)
    {
      float distance = (drone_position_ - launch_trajectory_endpoint_).norm();
      if (distance < 0.2f)
      {
        Vector3d home_point(5, 0, MAX_HEIGHT);
        // pub_pose_reference_.publish(generatePoseMsg(home_point, setOrientationFromTag(marker_position_)));
        waypoint_pub_.publish(generateWaypointMsg(home_point, setOrientationFromTag(marker_position_)));

        state_ = State::IDLE;
      }
    }

    if (computeBallRelease())
    {
      // pub_pose_reference_.publish(generatePoseMsg(drone_position_ + Vector3d(-1.0, 0.0, 0.1)));
      releaseBall();
      ball_released_ = true;
      // pub_pose_reference_.publish(generatePoseMsg(Vector3d(5, 0, MAX_HEIGHT)));
      // state_ = State::IDLE;

      // pub_pose_reference_.publish(generatePoseMsg(drone_position_ + Vector3d(-1.0, 0.0, 0.1)));
    }
    else
    {
      // POSE TRAJECTORY LAUNCHING
      // Vector3d wall_gap = identifyTagOrientation(marker_position_);
      float security_distance = 1.0f; // in meters
      launch_trajectory_endpoint_ = marker_position_ + tag_vector_ * security_distance;
      launch_trajectory_endpoint_.z() += launch_height_; // ADAPTED HEIGHT LAUNCH
      // launch_trajectory_endpoint_.z() = MAX_HEIGHT; // MAX HEIGHT LAUNCH

      // pub_pose_reference_.publish(generatePoseMsg(launch_trajectory_endpoint_, setOrientationFromTag(marker_position_)));
      waypoint_pub_.publish(generateWaypointMsg(launch_trajectory_endpoint_, setOrientationFromTag(marker_position_)));

      ROS_DEBUG_ONCE("launch_trajectory_endpoint_: %f, %f, %f",
                     launch_trajectory_endpoint_.x(),
                     launch_trajectory_endpoint_.y(),
                     launch_trajectory_endpoint_.z());

      // SPEED TRAJECTORY LAUNCHING
      // computeSpeedToFollow();
      // pub_speed_reference_.publish(speed_to_follow_);
    }
  }
  break;
  default:
    break;
  }
}

void BallThrowing::computeSpeedToFollow()
{
  // DIAGONAL LAUNCHING
  // Vector3d launch_trajectory_endpoint = marker_position_ + Vector3d(0.0, 0.0, MAX_HEIGHT);
  // Vector3d speed_to_follow_vector = (launch_trajectory_endpoint - drone_position_);

  // HORITZONTAL LAUNCHING
  Vector3d speed_to_follow_vector = (marker_position_ - drone_position_);
  speed_to_follow_vector.z() = 0;

  speed_to_follow_vector = speed_to_follow_vector.normalized() * LAUNCH_SPEED;

  speed_to_follow_.twist.linear.x = speed_to_follow_vector(0);
  speed_to_follow_.twist.linear.y = speed_to_follow_vector(1);
  speed_to_follow_.twist.linear.z = speed_to_follow_vector(2);
  speed_to_follow_.header.stamp = ros::Time::now();
}

// bool BallThrowing::computeBallRelease()
// {
//   if (drone_position_.x() > MAX_X || fabs(drone_position_.y()) > MAX_Y)
//   {
//     // LOG
//     ROS_INFO("Ball released because the drone is too close to the walls");
//     return true;
//   }

//   bool release = false;
//   Vector3d marker_position_diff = marker_position_ - drone_position_;
//   double z_diff = -marker_position_diff.z();
//   double z_speed = drone_velocity_.z();
//   // double z_speed = 0.0f;
//   double t_contact_z = T_DELAY + (z_speed + std::sqrt(z_speed * z_speed + 2 * 9.81f * z_diff)) / 9.81f;

//   // check it t_contact_z is nan
//   if (std::isnan(t_contact_z))
//   {
//     return false;
//   }
//   Vector3d marker_position_contact = drone_position_ + drone_velocity_ * t_contact_z;
//   marker_position_contact.z() = marker_position_.z();

//   double distance_to_marker = (marker_position_contact - marker_position_).norm();
//   // log
//   if (distance_to_marker < THROW_THRESHOLD)
//   {
//     release = true;
//   }
//   return release;
// }

bool BallThrowing::computeBallRelease()
{
  if (drone_position_.x() > MAX_X || fabs(drone_position_.y()) > MAX_Y)
  {
    // LOG
    ROS_INFO("Ball released because the drone is too close to the walls");
    return true;
  }

  bool release = false;

  Vector3d marker_position_diff = marker_position_ - drone_position_;
  marker_position_diff.z() = 0;

  double distance_to_marker = marker_position_diff.norm();
  // ROS_INFO("Distance to marker: %f", distance_to_marker);

  if (distance_to_marker < launch_distance_)
  {
    return true;
  }
  return false;
}

void BallThrowing::computeLaunchingParameters()
{
  float v_max = 3.0f;
  launch_height_ = 1.5f;

  float t_fall = std::sqrt(2 * launch_height_ / 9.81f);
  float t_launch = T_DELAY + t_fall;

  launch_distance_ = v_max * t_launch;

  // ROS_INFO("t_fall: %f", t_fall);
  // ROS_INFO("t_launch: %f", t_launch);
  ROS_DEBUG("Launch distance: %f", launch_distance_);
}

void BallThrowing::computeInitialPoint()
{
  // initial_point_ = marker_position_ - Vector3d(1, 0, 0) * BEGIN_POINT_DISTANCE_MARGIN;
  tag_vector_ = identifyTagOrientation(marker_position_);
  float launch_distance = 6.0f; // in meters
  initial_point_ = marker_position_ + tag_vector_ * launch_distance;

  // initial_point_.z() = MAX_HEIGHT; // MAX HEIGHT LAUNCH
  initial_point_.z() = marker_position_.z() + launch_height_; // ADAPTED HEIGHT LAUNCH

  // initial_point_.z() = BEGIN_POINT_HEIGHT; // DIAGONAL LAUNCHING
  ROS_DEBUG("Initial point: %f, %f, %f", initial_point_.x(), initial_point_.y(), initial_point_.z());
}

void BallThrowing::releaseBall()
{
  std_msgs::Float32 msg;
  msg.data = 0.0f;
  pub_release_ball_.publish(msg);
}

// Callbacks //

void BallThrowing::CallbackOdomTopic(const nav_msgs::Odometry &_odom_msg)
{
  if (state_ == State::IDLE)
  {
    state_ = State::WAITING_FOR_REFERENCES;
    // log
    std::cout << "Ball throwing: waiting for references" << std::endl;
  }
  drone_position_ = Vector3d(_odom_msg.pose.pose.position.x,
                             _odom_msg.pose.pose.position.y,
                             _odom_msg.pose.pose.position.z);
  drone_velocity_ = Vector3d(_odom_msg.twist.twist.linear.x,
                             _odom_msg.twist.twist.linear.y,
                             _odom_msg.twist.twist.linear.z);
}

void BallThrowing::CallbackTargetPositionTopic(const geometry_msgs::PoseStamped &_target_position_msg)
{
  // log
  if (state_ == State::WAITING_FOR_REFERENCES)
  {
    // log
    std::cout << "Ball throwing: received target position" << std::endl;
    state_ = State::APPROACHING_INITIAL_POINT;
    marker_position_ = Vector3d(_target_position_msg.pose.position.x + X_OFFSET_BALL,
                                _target_position_msg.pose.position.y,
                                _target_position_msg.pose.position.z + Z_OFFSET_BALL - 0.6f);
    computeInitialPoint();
    // pub_pose_reference_.publish(generatePoseMsg(initial_point_, setOrientationFromTag(marker_position_)));
    waypoint_pub_.publish(generateWaypointMsg(initial_point_, setOrientationFromTag(marker_position_)));
  }
}

// Messages //

geometry_msgs::PoseStamped BallThrowing::generatePoseMsg(
    const Eigen::Vector3d &_position,
    const Eigen::Quaterniond &_orientation)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.pose.position.x = _position.x();
  pose_msg.pose.position.y = _position.y();
  pose_msg.pose.position.z = _position.z();
  pose_msg.pose.orientation.x = _orientation.x();
  pose_msg.pose.orientation.y = _orientation.y();
  pose_msg.pose.orientation.z = _orientation.z();
  pose_msg.pose.orientation.w = _orientation.w();
  return pose_msg;
}

trajectory_msgs::MultiDOFJointTrajectoryPoint generateWaypointMsg(const Eigen::Vector3d &_position,
                                                                  const Eigen::Quaterniond &_orientation)
{

  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point;
  trajectory_point.transforms.resize(1);
  trajectory_point.transforms[0].translation.x = _position.x();
  trajectory_point.transforms[0].translation.y = _position.y();
  trajectory_point.transforms[0].translation.z = _position.z();
  trajectory_point.transforms[0].rotation.x = _orientation.x();
  trajectory_point.transforms[0].rotation.y = _orientation.y();
  trajectory_point.transforms[0].rotation.z = _orientation.z();
  trajectory_point.transforms[0].rotation.w = _orientation.w();
  trajectory_point.velocities.resize(1);
  // trajectory_point.velocities[0].x = 0;
  // trajectory_point.velocities[0].y = 0;
  // trajectory_point.velocities[0].z = 0;
  trajectory_point.accelerations.resize(1);
  // trajectory_point.accelerations[0].x = 0;
  // trajectory_point.accelerations[0].y = 0;
  // trajectory_point.accelerations[0].z = 0;
  trajectory_point.time_from_start = ros::Duration(0.1);
  return trajectory_point;
}

Eigen::Quaterniond setOrientationFromTag(const Eigen::Vector3d &_tag_position)
{
  float yaw = identifyTagYaw(_tag_position);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  const Eigen::Quaterniond orientation(q.w(), q.x(), q.y(), q.z());
  return orientation;
}

Vector3d identifyTagOrientation(const Vector3d &_tag_position)
{
  float x_max = 12.5f;
  float y_min = 7.5f;
  float confidence = 0.5f;

  if (_tag_position.x() > (x_max - confidence))
  {
    return Vector3d(-1, 0, 0);
  }
  else if (abs(_tag_position.y()) < (y_min - confidence))
  {
    return Vector3d(-1, 0, 0);
  }

  if (_tag_position.y() > 0)
  {
    return Vector3d(0, -1, 0);
  }
  else
  {
    return Vector3d(0, 1, 0);
  }
}

float identifyTagYaw(const Eigen::Vector3d &_tag_position)
{
  float x_max = 12.5f;
  float y_min = 7.5f;
  float confidence = 0.5f;

  if (_tag_position.x() > (x_max - confidence))
  {
    return 0.0f;
  }
  else if (abs(_tag_position.y()) < (y_min - confidence))
  {
    return 0.0f;
  }

  if (_tag_position.y() > 0)
  {
    return M_PI / 2;
  }
  else
  {
    return -M_PI / 2;
  }
}