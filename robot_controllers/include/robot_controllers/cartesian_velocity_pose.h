/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  [License Text Unchanged]
 *********************************************************************/

/*
 * Derived a bit from pr2_controllers/cartesian_pose_controller.cpp
 * Author: Michael Ferguson, Wim Meeussen
 */

#ifndef ROBOT_CONTROLLERS_CARTESIAN_VEL_POSE_H
#define ROBOT_CONTROLLERS_CARTESIAN_VEL_POSE_H

#include <string>
#include <vector>
#include <memory>  // Changed from boost::shared_ptr to std::shared_ptr

#include <ros/ros.h>
#include <robot_controllers/pid.h>
#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/joint_handle.h>
#include <robot_controllers_interface/controller_manager.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>  // Added for inverse velocity solver
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace robot_controllers
{

class CartesianPoseVelController : public Controller
{
public:
  CartesianPoseVelController();
  virtual ~CartesianPoseVelController() {}

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
    return "robot_controllers/CartesianPoseVelController";
  }

  /** @brief Get the names of joints/controllers which this controller commands. */
  virtual std::vector<std::string> getCommandedNames();

  /** @brief Get the names of joints/controllers which this controller exclusively claims. */
  virtual std::vector<std::string> getClaimedNames();

  /** @brief Controller command. */
  void command(const geometry_msgs::PoseStamped::ConstPtr& goal);

private:
  KDL::Frame getPose();

  bool initialized_;
  ControllerManager* manager_;

  bool enabled_;
  std::string root_link_;
  ros::Time last_command_;

  KDL::Frame desired_pose_;
  KDL::Frame actual_pose_;

  KDL::Twist twist_error_;

  KDL::Chain kdl_chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;  // Added inverse velocity solver

  KDL::JntArray jnt_pos_;
  KDL::JntArray jnt_vel_;        // Added joint velocities
  KDL::Jacobian jacobian_;

  ros::Publisher feedback_pub_;
  ros::Subscriber command_sub_;

  tf::TransformListener tf_;
  std::vector<JointHandlePtr> joints_;
  std::vector<robot_controllers::PID> pid_;

  // Additional member variables if needed
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS_CARTESIAN_VEL_POSE_H