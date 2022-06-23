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
  path_facing_pub_ = nh_.advertise<PATH_FACING_TOPIC_TYPE>(PATH_FACING_TOPIC, 1);

  // Load parameters
  nh_.getParam("ball_throwing/launch_speed", launch_speed_);
  nh_.getParam("ball_throwing/launch_height", launch_height_);
  nh_.getParam("ball_throwing/launch_start_distance", launch_start_distance_);
  nh_.getParam("ball_throwing/speed_reference", speed_controller_);

  nh_.getParam("ball_throwing/x_offset_ball", x_offset_ball_);
  nh_.getParam("ball_throwing/y_offset_ball", y_offset_ball_);
  nh_.getParam("ball_throwing/z_offset_ball", z_offset_ball_);
  nh_.getParam("ball_throwing/z_correction", z_correction_);
  nh_.getParam("ball_throwing/t_delay", t_delay_);

  nh_.getParam("ball_throwing/launch_security_distance", launch_security_distance_);
  nh_.getParam("ball_throwing/map_max_x", map_max_x_);
  nh_.getParam("ball_throwing/map_max_y", map_max_y_);
  nh_.getParam("ball_throwing/map_max_z", map_max_z_);

  nh_.getParam("ball_throwing/home_x", home_point_.x());
  nh_.getParam("ball_throwing/home_y", home_point_.y());
  nh_.getParam("ball_throwing/home_z", home_point_.z());

  nh_.getParam("ball_throwing/ball_radious", ball_radious_);
  nh_.getParam("ball_throwing/throw_threshold", throw_threshold_);
  nh_.getParam("ball_throwing/begin_point_distance_margin", begin_point_distance_margin_);
  nh_.getParam("ball_throwing/begin_point_height", begin_point_height_);

  ROS_INFO("x_offset_ball: %.2f", x_offset_ball_);
  ROS_INFO("y_offset_ball: %.2f", y_offset_ball_);
  ROS_INFO("z_offset_ball: %.2f", z_offset_ball_);
  ROS_INFO("z_correction: %.2f", z_correction_);
  ROS_INFO("t_delay: %.2f", t_delay_);

  ROS_INFO("Launch security distance: %.2f", launch_security_distance_);
  ROS_INFO("Map limits: x: %.2f, y: %.2f, z: %.2f", map_max_x_, map_max_y_, map_max_z_);
  ROS_INFO("Home: x: %.2f, y: %.2f, z: %.2f", home_point_.x(), home_point_.y(), home_point_.z());

  ROS_INFO("ball_radious: %.2f", ball_radious_);
  ROS_INFO("throw_threshold: %.2f", throw_threshold_);
  ROS_INFO("begin_point_distance_margin: %.2f", begin_point_distance_margin_);
  ROS_INFO("begin_point_height: %.2f", begin_point_height_);

  ROS_INFO("Launch speed: %.2f", launch_speed_);
  ROS_INFO("Launch height: %.2f", launch_height_);
  ROS_INFO("Launch start distance: %.2f", launch_start_distance_);

  std::string controller_str = "pose reference";
  if (speed_controller_)
  {
    controller_str = "speed controller";
  }
  ROS_INFO("Controller mode: %s", controller_str.c_str());

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
      std_msgs::Bool path_facing_msg_;
      path_facing_msg_.data = false;
      path_facing_pub_.publish(path_facing_msg_);
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    else
    {
      pub_pose_reference_.publish(generatePoseMsg(initial_point_, setOrientationFromTag(marker_position_)));
      // waypoint_pub_.publish(generateWaypointMsg(initial_point_, setOrientationFromTag(marker_position_)));
    }
  }
  break;
  case State::THROWING_TRAJECTORY:
  {
    if (ball_released_)
    {
      // float distance = (drone_position_ - launch_trajectory_endpoint_).norm();
      // if (distance < 0.2f)
      // {
      // Vector3d home_point(5, 0, max_height_);
      ROS_INFO("Ball thrown, going to home point: %.2f, %.2f, %.2f", home_point_.x(), home_point_.y(), home_point_.z());

      // std::this_thread::sleep_for(std::chrono::milliseconds(100));

      pub_pose_reference_.publish(generatePoseMsg(home_point_, setOrientationFromTag(marker_position_)));
      // waypoint_pub_.publish(generateWaypointMsg(home_point_, setOrientationFromTag(marker_position_)));

      state_ = State::IDLE;
      // }
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

      if (speed_controller_)
      {
        computeSpeedToFollow();
        pub_speed_reference_.publish(speed_to_follow_);
      }
      else
      {
        // POSE TRAJECTORY LAUNCHING
        //
        // Vector3d wall_gap = identifyTagOrientation(marker_position_);
        // security_distance_ = 1.0f; // in meters
        launch_trajectory_endpoint_ = marker_position_ + tag_vector_ * launch_security_distance_;
        launch_trajectory_endpoint_.z() += launch_height_; // ADAPTED HEIGHT LAUNCH
        // launch_trajectory_endpoint_.z() = MAX_HEIGHT; // MAX HEIGHT LAUNCH
        if (launch_trajectory_endpoint_.z() > map_max_z_)
        {
          launch_trajectory_endpoint_.z() = map_max_z_;
        }
        pub_pose_reference_.publish(generatePoseMsg(launch_trajectory_endpoint_, setOrientationFromTag(marker_position_)));
        // waypoint_pub_.publish(generateWaypointMsg(launch_trajectory_endpoint_, setOrientationFromTag(marker_position_)));

        ROS_DEBUG_ONCE("launch_trajectory_endpoint_: %f, %f, %f",
                       launch_trajectory_endpoint_.x(),
                       launch_trajectory_endpoint_.y(),
                       launch_trajectory_endpoint_.z());
      }

      // SPEED TRAJECTORY LAUNCHING
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

  speed_to_follow_vector = speed_to_follow_vector.normalized() * launch_speed_;

  speed_to_follow_.twist.linear.x = speed_to_follow_vector(0);
  speed_to_follow_.twist.linear.y = speed_to_follow_vector(1);
  // speed_to_follow_.twist.linear.z = speed_to_follow_vector(2);
  speed_to_follow_.twist.linear.z = 0.1f;
  speed_to_follow_.header.stamp = ros::Time::now();
}

bool BallThrowing::computeBallRelease()
{
  if (drone_position_.x() > map_max_x_ || fabs(drone_position_.y()) > map_max_y_)
  {
    // LOG
    ROS_INFO("Ball released because the drone is too close to the walls");
    return true;
  }

  bool release = false;
  Vector3d marker_position_diff = marker_position_ - drone_position_;
  double z_diff = -marker_position_diff.z();
  double z_speed = drone_velocity_.z();
  // double z_speed = 0.0f;
  double t_contact_z = (z_speed + std::sqrt(z_speed * z_speed + 2 * 9.81f * z_diff)) / 9.81f;

  ROS_INFO("T contact: %.4f", t_contact_z);

  // check it t_contact_z is nan
  if (std::isnan(t_contact_z))
  {
    return false;
  }
  Vector3d marker_position_contact = drone_position_ + drone_velocity_ * t_contact_z;
  std::cout << "marker_position_contact: " << marker_position_contact.transpose() << std::endl;
  // marker_position_contact.z() = marker_position_.z();

  // double distance_to_marker = (marker_position_contact - marker_position_).norm();
  // log
  // if (distance_to_marker < throw_threshold_)
  if (marker_position_contact.x() > marker_position_.x() - throw_threshold_)
  {
    ROS_WARN("Releasing ball for contact at [%.2f, %.2f, %.2f]", marker_position_contact.x(), marker_position_contact.y(), marker_position_contact.z());
    release = true;
  }
  return release;
}

// bool BallThrowing::computeBallRelease()
// {
//   if (drone_position_.x() > (map_max_x_ - launch_security_distance_) ||
//       fabs(drone_position_.y()) > (map_max_y_ - launch_security_distance_))
//   {
//     // LOG
//     ROS_INFO("Ball released because the drone is too close to the walls");
//     return true;
//   }

//   bool release = false;

//   Vector3d marker_position_diff = marker_position_ - drone_position_;
//   marker_position_diff.z() = 0;

//   double distance_to_marker = marker_position_diff.norm();
//   // ROS_INFO("Distance to marker: %f", distance_to_marker);

//   if (distance_to_marker < launch_distance_)
//   {
//     return true;
//   }
//   return false;
// }

void BallThrowing::computeLaunchingParameters()
{
  float v_max = launch_speed_;
  // launch_height_ = 1.5f;

  float t_fall = std::sqrt(2 * launch_height_ / 9.81f);
  float t_launch = t_delay_ + t_fall;

  launch_distance_ = v_max * t_launch;

  // ROS_INFO("t_fall: %f", t_fall);
  // ROS_INFO("t_launch: %f", t_launch);
  ROS_DEBUG("Launch distance: %f", launch_distance_);
}

void BallThrowing::computeInitialPoint()
{
  // initial_point_ = marker_position_ - Vector3d(1, 0, 0) * BEGIN_POINT_DISTANCE_MARGIN;
  tag_vector_ = identifyTagOrientation(marker_position_);
  // float launch_distance = 6.0f; // in meters
  initial_point_ = marker_position_ + tag_vector_ * launch_start_distance_;

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
    marker_position_ = Vector3d(_target_position_msg.pose.position.x + x_offset_ball_,
                                _target_position_msg.pose.position.y + y_offset_ball_,
                                _target_position_msg.pose.position.z + z_offset_ball_ + z_correction_);
    computeInitialPoint();
    pub_pose_reference_.publish(generatePoseMsg(initial_point_, setOrientationFromTag(marker_position_)));
    // waypoint_pub_.publish(generateWaypointMsg(initial_point_, setOrientationFromTag(marker_position_)));
  }
  else
  {
    ROS_INFO("QUEEEEEEEEEEEEEEEE");
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

Eigen::Quaterniond BallThrowing::setOrientationFromTag(const Eigen::Vector3d &_tag_position)
{
  float yaw = identifyTagYaw(_tag_position);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  const Eigen::Quaterniond orientation(q.w(), q.x(), q.y(), q.z());
  return orientation;
}

Vector3d BallThrowing::identifyTagOrientation(const Vector3d &_tag_position)
{
  float x_max = map_max_x_;
  float y_min = map_max_y_;
  float confidence = identify_tag_wall_confidence_;

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

float BallThrowing::identifyTagYaw(const Eigen::Vector3d &_tag_position)
{
  float x_max = map_max_x_;
  float y_min = map_max_y_;
  float confidence = identify_tag_wall_confidence_;

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
