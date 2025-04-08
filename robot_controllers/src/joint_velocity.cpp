/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Fetch Robotics Inc.
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Unbounded Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
* Derived a bit from pr2_controllers/cartesian_pose_controller.cpp
* Author: Michael Ferguson, Wim Meeussen
*/

#include <pluginlib/class_list_macros.hpp>
#include <robot_controllers/joint_velocity.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::JointVelocityController, robot_controllers::Controller)

namespace robot_controllers
{

JointVelocityController::JointVelocityController() :
    initialized_(false),
    enabled_(false)
{
}

int JointVelocityController::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  // Ensure access to the controller manager
  if (!manager)
  {
    initialized_ = false;
    return -1;
  }

  Controller::init(nh, manager);
  manager_ = manager;

  // Get joint names from parameter server
  if (!nh.getParam("joints", joint_names_))
  {
    ROS_ERROR("No joints specified for velocity controller");
    return -1;
  }

  // Initialize joint handles
  joints_.clear();
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    JointHandlePtr joint = manager_->getJointHandle(joint_names_[i]);
    if (!joint)
    {
      ROS_ERROR_STREAM("Failed to get joint handle for " << joint_names_[i]);
      return -1;
    }
    joints_.push_back(joint);
  }

  // Initialize commanded velocities
  commanded_velocities_.resize(joints_.size(), 0.0);
  actual_velocities_.resize(joints_.size(), 0.0);
  
  // Initialize PID controllers for each joint
  pid_controllers_.clear();
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    robot_controllers::PID pid_controller;
    // Try to load joint-specific PID parameters first
    std::string param_prefix = "pid_" + joint_names_[i];
    if (nh.hasParam(param_prefix))
    {
      if (!pid_controller.init(ros::NodeHandle(nh, param_prefix)))
      {
        ROS_ERROR_STREAM("Failed to initialize PID controller for " << joint_names_[i]);
        return -1;
      }
    }
    else
    {
      // Fall back to common PID parameters
      if (!pid_controller.init(ros::NodeHandle(nh, "pid")))
      {
        ROS_ERROR("Failed to initialize PID controller with common parameters");
        return -1;
      }
    }
    pid_controllers_.push_back(pid_controller);
  }

  // Subscribe to velocity commands
  command_sub_ = nh.subscribe<sensor_msgs::JointState>(
      "command", 1, &JointVelocityController::velocityCommand, this);

  initialized_ = true;
  last_state_.resize(joints_.size(), 0.0);
  return 0;
}

void JointVelocityController::update(const ros::Time& now, const ros::Duration& dt)
{
  if (!initialized_ || !enabled_)
    return;

  // Check command timeout
  if ((now - last_command_).toSec() > 0.3)
  {
    // Stop joints if no recent command
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      // joints_[i]->setPosition(joints_[i]->getPosition(), 0.0, 0.0);
      // joints_[i]->setVelocity(0.0, 0);
      joints_[i]->setPosition(last_state_[i], 0.0, 0.0);
      // joints_[i]->setPosition(last_state_[i], 0.0, 0.0);
    }
    return;
  }

  // Update actual velocities
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    actual_velocities_[i] = joints_[i]->getVelocity();
  }

  // Apply PID control to achieve commanded velocities
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    // Calculate velocity error
    double velocity_error = commanded_velocities_[i] - actual_velocities_[i];
    
    // Get effort from PID controller
    double effort = pid_controllers_[i].update(velocity_error, dt.toSec());
    
    // Apply velocity with computed effort
    joints_[i]->setVelocity(commanded_velocities_[i], effort);
    
    // Store current position for use when stopping
    last_state_[i] = joints_[i]->getPosition();
  }
}

void JointVelocityController::velocityCommand(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (!initialized_)
    return;

  // Update last command time
  last_command_ = ros::Time::now();
  
  // Try to start controller if not already running
  if (!enabled_)
  {
    if (manager_->requestStart(getName()) != 0)
    {
      ROS_ERROR("JointVelocityController: Cannot start controller");
      return;
    }
    enabled_ = true;
  }

  // Update commanded velocities
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    // Find the joint in our list
    for (size_t j = 0; j < joint_names_.size(); ++j)
    {
      if (msg->name[i] == joint_names_[j])
      {
        commanded_velocities_[j] = msg->velocity[i];
        break;
      }
    }
  }
}

bool JointVelocityController::start()
{
  if (!initialized_)
  {
    ROS_ERROR_NAMED("JointVelocityController",
                    "Unable to start, not initialized.");
    return false;
  }

  // Reset PIDs to avoid accumulated error
  for (size_t i = 0; i < pid_controllers_.size(); ++i)
  {
    pid_controllers_[i].reset();
  }

  if (ros::Time::now() - last_command_ > ros::Duration(3.0))
  {
    ROS_ERROR_NAMED("JointVelocityController",
                    "Unable to start, no goal.");
    return false;
  }

  enabled_ = true;
  return true;
}

bool JointVelocityController::stop(bool force)
{
  // Reset PIDs when stopping
  for (size_t i = 0; i < pid_controllers_.size(); ++i)
  {
    pid_controllers_[i].reset();
  }
  
  enabled_ = false;
  // Always stop
  return true;
}

bool JointVelocityController::reset()
{
  // Reset PIDs
  for (size_t i = 0; i < pid_controllers_.size(); ++i)
  {
    pid_controllers_[i].reset();
  }
  
  // Simply stop
  return (manager_->requestStop(getName()) == 0);
}

std::vector<std::string> JointVelocityController::getCommandedNames()
{
  return joint_names_;
}

std::vector<std::string> JointVelocityController::getClaimedNames()
{
  return joint_names_;
}

}  // namespace robot_controllers
