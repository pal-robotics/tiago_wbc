#include <gtest/gtest.h>
#include <ros/ros.h>
#include <pal_wbc_msgs/GetStackDescription.h>
#include <pal_wbc_msgs/PopTask.h>
#include <pal_wbc_msgs/PushTask.h>
#include <pal_wbc_msgs/GetTaskError.h>
#include <property_bag/property_bag.h>
#include <Eigen/Dense>
#include <pal_wbc_utils/pal_wbc_utils.h>
#include <pal_wbc_msgs/PushPopTask.h>

using namespace pal;

class WBCTests : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    stackDescriptionServ_ = nh_.serviceClient<pal_wbc_msgs::GetStackDescription>(
        "whole_body_kinematic_controller/get_stack_description");
    popTaskServ_ =
        nh_.serviceClient<pal_wbc_msgs::PopTask>("whole_body_kinematic_controller/pop_task");
    pushTaskServ_ =
        nh_.serviceClient<pal_wbc_msgs::PushTask>("whole_body_kinematic_controller/push_task");
    getTaskErrorServ_ = nh_.serviceClient<pal_wbc_msgs::GetTaskError>(
        "whole_body_kinematic_controller/get_task_error");
//    pushPopTaskServ_ = nh_.serviceClient<pal_wbc_msgs::PushPopTask>(
//        "whole_body_kinematic_controller/push_pop_task");

    ros::Duration timeout = ros::Duration(10.0);

    if (!stackDescriptionServ_.waitForExistence(timeout))
      FAIL() << "Service get stack not ready.";
    if (!popTaskServ_.waitForExistence(timeout))
      FAIL() << "Service pop task not ready.";
    if (!pushTaskServ_.waitForExistence(timeout))
      FAIL() << "Service push task not ready.";
    if (!getTaskErrorServ_.waitForExistence(timeout))
      FAIL() << "Service get task error not ready.";
//    if (!pushPopTaskServ_.waitForExistence(timeout))
//      FAIL() << "Service not ready.";

    blend_ = false;
    reference_ = "pointer_reflexx_typeII";
    tip_name_ = "arm_tool_link";
    base_frame_ = "base_footprint";

    positionGoal_ << 0.64637, -0.13707, 0.95628;
    positionGoal2_ << 0.54637, -0.20707, 0.40628;
    positionGoal3_ << 0.44637, -0.20707, 0.85628;

    orientationGoal_.w() = 1.0;
    orientationGoal_.x() = 0.0;
    orientationGoal_.y() = 0.0;
    orientationGoal_.z() = 0.0;

    orientationGoal2_.w() = 0.0;
    orientationGoal2_.x() = 1.0;
    orientationGoal2_.y() = 0.0;
    orientationGoal2_.z() = 0.0;
  }

  bool stack_description_contains(const std::string &task)
  {
    pal_wbc_msgs::GetStackDescription statusSrv;
    if (!stackDescriptionServ_.call(statusSrv))
      throw std::runtime_error("Error in stack_description_contains");

    for (auto t : statusSrv.response.tasks)
    {
      if (t.name == task)
      {
        return true;
      }
    }
    return false;
  }

  ros::NodeHandle nh_;
  ros::ServiceClient stackDescriptionServ_;
  ros::ServiceClient popTaskServ_;
  ros::ServiceClient pushTaskServ_;
  ros::ServiceClient getTaskErrorServ_;
//  ros::ServiceClient pushPopTaskServ_;
  bool blend_;
  std::string reference_;
  std::string tip_name_;
  std::string base_frame_;

  Eigen::Vector3d positionGoal_;
  Eigen::Vector3d positionGoal2_;
  Eigen::Vector3d positionGoal3_;
  Eigen::Quaterniond orientationGoal_;
  Eigen::Quaterniond orientationGoal2_;
};
