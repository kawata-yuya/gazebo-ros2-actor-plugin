/*
 * Copyright 2023 blackcoffeerobotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * Modifications copyright (C) 2025 kawata-yuya
 * Changes: Upgraded code to support ROS2 Humble.
*/

#include <gazebo_ros2_actor_plugin/gazebo_ros2_actor_command.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <functional>

#include "gazebo/physics/physics.hh"
#include <ignition/math.hh>

using namespace gazebo;

#define _USE_MATH_DEFINES
#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
GazeboRosActorCommand::GazeboRosActorCommand()
{
  gzmsg << "GazeboRosActorCommand initializing\n";
}

GazeboRosActorCommand::~GazeboRosActorCommand()
{
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Set default values for parameters
  this->follow_mode_ = "velocity";
  this->vel_topic_ = "/cmd_vel";
  this->path_topic_ = "/cmd_path";
  this->lin_tolerance_ = 0.1;
  this->lin_velocity_ = 1;
  this->ang_tolerance_ = IGN_DTOR(5);
  this->ang_velocity_ = IGN_DTOR(10);
  this->animation_factor_ = 4.0;

  // Override default parameter values with values from SDF
  if (_sdf->HasElement("follow_mode")) {
    this->follow_mode_ = _sdf->Get<std::string>("follow_mode");
  }
  if (_sdf->HasElement("vel_topic")) {
    this->vel_topic_ = _sdf->Get<std::string>("vel_topic");
  }
  if (_sdf->HasElement("path_topic")) {
    this->path_topic_ = _sdf->Get<std::string>("path_topic");
  }
  if (_sdf->HasElement("linear_tolerance")) {
    this->lin_tolerance_ = _sdf->Get<double>("linear_tolerance");
  }
  if (_sdf->HasElement("linear_velocity")) {
    this->lin_velocity_ = _sdf->Get<double>("linear_velocity");
  }
  if (_sdf->HasElement("angular_tolerance")) {
    this->ang_tolerance_ = _sdf->Get<double>("angular_tolerance");
  }
  if (_sdf->HasElement("angular_velocity")) {
    this->ang_velocity_ = _sdf->Get<double>("angular_velocity");
  }
  if (_sdf->HasElement("animation_factor")) {
    this->animation_factor_ = _sdf->Get<double>("animation_factor");
  }
  if (_sdf->HasElement("default_rotation")) {
    this->default_rotation_ = _sdf->Get<double>("default_rotation");
  }
  

  // Set variables
  this->sdf_ = _sdf;
  this->actor_ = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world_ = this->actor_->GetWorld();
  this->Reset();
  // Create ROS2 node handle
  this->ros_node_ = gazebo_ros::Node::Get(this->sdf_);

  // Subscribe to the velocity commands
  this->vel_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    vel_topic_, rclcpp::QoS(10),
    std::bind(&GazeboRosActorCommand::VelCallback, this, std::placeholders::_1));

  // Subscribe to the path commands
  this->path_sub_ = this->ros_node_->create_subscription<nav_msgs::msg::Path>(
    path_topic_, rclcpp::QoS(10),
    std::bind(&GazeboRosActorCommand::PathCallback, this, std::placeholders::_1));

  // Connect the OnUpdate function to the WorldUpdateBegin event.
  this->connections_.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosActorCommand::OnUpdate, this, std::placeholders::_1)));
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::Reset() {
  // Reset last update time and target pose index
  this->last_update_ = 0;
  this->idx_ = 0;
  // Initialize target poses vector with origin
  this->target_poses_.push_back(ignition::math::Vector3d(0.0, 0.0, 0.0));
  // Set target pose to the current pose
  this->target_pose_ = this->target_poses_.at(this->idx_);

  // Check if the walking animation exists in the actor's skeleton animations
  auto skelAnims = this->actor_->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end()) {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  } else {
    // Create custom trajectory
    this->trajectoryInfo_.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo_->type = WALKING_ANIMATION;
    this->trajectoryInfo_->duration = 1.0;

    // Set the actor's trajectory to the custom trajectory
    this->actor_->SetCustomTrajectory(this->trajectoryInfo_);
  }
}

void GazeboRosActorCommand::VelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  ignition::math::Vector3d vel_cmd;
  vel_cmd.X() = msg->linear.x;
  vel_cmd.Z() = msg->angular.z;
  this->cmd_queue_.push(vel_cmd);
}

void GazeboRosActorCommand::PathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  // Extract the poses from the Path message
  const std::vector<geometry_msgs::msg::PoseStamped>& poses = msg->poses;

  // Extract the x, y, and yaw from each pose and store it as a target
  for (size_t i = 0; i < poses.size(); ++i) {
    const geometry_msgs::msg::Pose& pose = poses[i].pose;
    const double x = pose.position.x;
    const double y = pose.position.y;

    // Convert quaternion to Euler angles
    tf2::Quaternion quat(pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    this->target_poses_.push_back(ignition::math::Vector3d(x, y, yaw));
  }
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::OnUpdate(const common::UpdateInfo &_info) {
  // Time delta
  double dt = (_info.simTime - this->last_update_).Double();
  ignition::math::Pose3d pose = this->actor_->WorldPose();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  if (this->follow_mode_ == "path") {
    ignition::math::Vector2d target_pos_2d(this->target_pose_.X(),
                                           this->target_pose_.Y());
    ignition::math::Vector2d current_pos_2d(pose.Pos().X(),
                                            pose.Pos().Y());
    ignition::math::Vector2d pos = target_pos_2d - current_pos_2d;
    double distance = pos.Length();

    // Check if actor has reached current target position
    if (distance < this->lin_tolerance_) {
      // If there are more targets, choose new target
      if (this->idx_ < this->target_poses_.size() - 1) {
      this->ChooseNewTarget();
      pos.X() = this->target_pose_.X() - pose.Pos().X();
      pos.Y() = this->target_pose_.Y() - pose.Pos().Y();
      } else {
        // All targets have been accomplished, stop moving
        pos.X() = 0;
        pos.Y() = 0;
      }
    }

    // Normalize the direction vector
    if (pos.Length() != 0) {
      pos = pos/pos.Length();
    }

    int rot_sign = 1;
    // Calculate the angular displacement required based on the direction
    // vector towards the current target position
    ignition::math::Angle yaw(0);
    if (pos.Length() != 0) {
      yaw = atan2(pos.Y(), pos.X()) + default_rotation_ - rpy.Z();
      yaw.Normalize();
    }
    if (yaw < 0)
      rot_sign = -1;
    // Check if required angular displacement is greater than tolerance
    if (std::abs(yaw.Radian()) > this->ang_tolerance_) {

      pose.Rot() = ignition::math::Quaterniond(default_rotation_, 0, rpy.Z()+
            rot_sign*this->ang_velocity_ * dt);
    } 
    else {
        // Move towards the target position
        pose.Pos().X() += pos.X() * this->lin_velocity_ * dt;
        pose.Pos().Y() += pos.Y() * this->lin_velocity_ * dt;

        pose.Rot() = ignition::math::Quaterniond(
          default_rotation_, 0, rpy.Z()+yaw.Radian());
    }
  }
  else if (this->follow_mode_ == "velocity") {
    if (!this->cmd_queue_.empty()) {
      this->target_vel_.Pos().X() = this->cmd_queue_.front().X();
      this->target_vel_.Rot() = ignition::math::Quaterniond(
        0, 0, this->cmd_queue_.front().Z());
      this->cmd_queue_.pop();
    }

    pose.Pos().X() += this->target_vel_.Pos().X() *
                      cos(pose.Rot().Euler().Z() - default_rotation_) * dt;
    pose.Pos().Y() += this->target_vel_.Pos().X() *
                      sin(pose.Rot().Euler().Z() - default_rotation_) * dt;

    pose.Rot() = ignition::math::Quaterniond(
      default_rotation_, 0, rpy.Z()+this->target_vel_.Rot().Euler().Z()*dt);
  }

  // Distance traveled is used to coordinate motion with the walking animation
  auto displacement = pose.Pos() - this->actor_->WorldPose().Pos();
  double distanceTraveled = displacement.Length();

  this->actor_->SetWorldPose(pose, false, false);
  this->actor_->SetScriptTime(
  this->actor_->ScriptTime() + (distanceTraveled * this->animation_factor_));
  this->last_update_ = _info.simTime;
}

void GazeboRosActorCommand::ChooseNewTarget() {
  this->idx_++;

  // Set next target
  this->target_pose_ = this->target_poses_.at(this->idx_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosActorCommand)