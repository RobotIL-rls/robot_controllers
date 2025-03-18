/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  [License Text Unchanged]
 *********************************************************************/

/*
 * Derived a bit from pr2_controllers/cartesian_pose_controller.cpp
 * Author: Michael Ferguson, Wim Meeussen
 */

#ifndef ROBOT_CONTROLLERS_JOINT_VELOCITY_H
#define ROBOT_CONTROLLERS_JOINT_VELOCITY_H

#include <string>
#include <vector>
#include <memory>

#include <ros/ros.h>
#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/joint_handle.h>
#include <robot_controllers_interface/controller_manager.h>

#include <sensor_msgs/JointState.h>

namespace robot_controllers
{

class JointVelocityController : public Controller
{
public:
  JointVelocityController();
  virtual ~JointVelocityController() {}

  /**
   * @brief Initialize the controller and any required data structures.
   * @param nh Node handle for this controller.
   * @param manager The controller manager instance, this is needed for the
   *        controller to get information about joints, etc.
   * @returns 0 if successfully configured, negative values are error codes.
   */
  virtual int init(ros::NodeHandle& nh, ControllerManager* manager);

  /**
   * @brief Attempt to start the controller. This should be called only by the
   *        ControllerManager instance.
   * @returns True if successfully started, false otherwise.
   */
  virtual bool start();

  /**
   * @brief Attempt to stop the controller. This should be called only by the
   *        ControllerManager instance.
   * @param force Should we force the controller to stop? Some controllers
   *        may wish to continue running until they absolutely have to stop.
   * @returns True if successfully stopped, false otherwise.
   */
  virtual bool stop(bool force);

  /**
   * @brief Cleanly reset the controller to its initial state. Some controllers
   *        may choose to stop themselves. This is mainly used in the case of the
   *        robot exiting some fault condition.
   * @returns True if successfully reset, false otherwise.
   */
  virtual bool reset();

  /**
   * @brief This is the update loop for the controller.
   * @param time The system time.
   * @param dt The timestep since last call to update.
   */
  virtual void update(const ros::Time& now, const ros::Duration& dt);

  /** @brief Get the type of this controller. */
  virtual std::string getType()
  {
    return "robot_controllers/JointVelocityController";
  }

  /** @brief Get the names of joints/controllers which this controller commands. */
  virtual std::vector<std::string> getCommandedNames();

  /** @brief Get the names of joints/controllers which this controller exclusively claims. */
  virtual std::vector<std::string> getClaimedNames();

private:
  void velocityCommand(const sensor_msgs::JointState::ConstPtr& msg);

  bool initialized_;
  ControllerManager* manager_;
  bool enabled_;
  ros::Time last_command_;

  ros::Subscriber command_sub_;
  std::vector<JointHandlePtr> joints_;
  std::vector<double> commanded_velocities_;
  std::vector<std::string> joint_names_;
  std::vector<double> last_state_;
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS_JOINT_VELOCITY_H