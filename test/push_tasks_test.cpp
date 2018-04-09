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
    pushPopTaskServ_ = nh_.serviceClient<pal_wbc_msgs::PushPopTask>(
        "whole_body_kinematic_controller/push_pop_task");

    ros::Duration timeout = ros::Duration(10.0);

    if (!stackDescriptionServ_.waitForExistence(timeout))
      FAIL() << "Service not ready.";
    if (!popTaskServ_.waitForExistence(timeout))
      FAIL() << "Service not ready.";
    if (!pushTaskServ_.waitForExistence(timeout))
      FAIL() << "Service not ready.";
    if (!getTaskErrorServ_.waitForExistence(timeout))
      FAIL() << "Service not ready.";
    if (!pushPopTaskServ_.waitForExistence(timeout))
      FAIL() << "Service not ready.";

    blend_ = false;
    reference_ = "pointer_reflexx_typeII";
    tip_name_ = "arm_tool_link";
  }

  ros::NodeHandle nh_;
  ros::ServiceClient stackDescriptionServ_;
  ros::ServiceClient popTaskServ_;
  ros::ServiceClient pushTaskServ_;
  ros::ServiceClient getTaskErrorServ_;
  ros::ServiceClient pushPopTaskServ_;
  bool blend_;
  std::string reference_;
  std::string tip_name_;
};

TEST_F(WBCTests, GetStackDescription)
{
  pal_wbc_msgs::GetStackDescription statusSrv;
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
}

TEST_F(WBCTests, PushingPositionTask)
{
  property_bag::PropertyBag taskProperties("taskType",
                                           std::string("pal_wbc/GoToPositionMetaTask"));

  Eigen::Vector3d positionGoal(0.74637, -0.13707, 0.95628);

  taskProperties.addProperty("task_id", std::string("go_to_position"));
  taskProperties.addProperty("target_position", positionGoal);
  taskProperties.addProperty("tip_name", tip_name_);
  taskProperties.addProperty("damping", 0.2);
  taskProperties.addProperty("signal_reference", reference_);

  pal_wbc_msgs::PushTask srv;
  srv.request.push_task_params.params = generateTaskDescription(taskProperties);
  srv.request.push_task_params.respect_task_id = "self_collision";
  srv.request.push_task_params.order.order = pal_wbc_msgs::Order::After;
  srv.request.push_task_params.blend = blend_;

  EXPECT_TRUE(pushTaskServ_.call(srv));
}

TEST_F(WBCTests, GetStackDescription2)
{
  ros::Duration(1.0).sleep();
  pal_wbc_msgs::GetStackDescription statusSrv;
  bool find = false;
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  for (auto t : statusSrv.response.tasks)
  {
    if (t.name == "go_to_position")
    {
      find = true;
      break;
    }
  }
  EXPECT_TRUE(find);
}

TEST_F(WBCTests, GetTaskError)
{
  ros::Duration(4.0).sleep();
  pal_wbc_msgs::GetTaskError srv;
  srv.request.id = "go_to_position";
  EXPECT_TRUE(getTaskErrorServ_.call(srv));
  EXPECT_LT(srv.response.taskError.error_norm, 1.e-2);
}

TEST_F(WBCTests, PushPositionTask2)
{
  property_bag::PropertyBag taskProperties("taskType",
                                           std::string("pal_wbc/GoToPositionMetaTask"));

  Eigen::Vector3d positionGoal(0.54637, -0.13707, 0.95628);

  taskProperties.addProperty("task_id", std::string("new_go_to_position"));
  taskProperties.addProperty("target_position", positionGoal);
  taskProperties.addProperty("tip_name", std::string("arm_tool_link"));
  taskProperties.addProperty("damping", 0.2);
  taskProperties.addProperty("signal_reference", reference_);

  pal_wbc_msgs::PushPopTask srv;
  pal_wbc_msgs::PushTaskParams push_params;
  push_params.params = generateTaskDescription(taskProperties);
  push_params.respect_task_id = "go_to_position";
  push_params.order.order = pal_wbc_msgs::Order::Replace;
  push_params.blend = blend_;
  srv.request.push_tasks.push_back(push_params);
  EXPECT_TRUE(pushPopTaskServ_.call(srv));
}

TEST_F(WBCTests, GetStackDescription3)
{
  ros::Duration(1.0).sleep();
  pal_wbc_msgs::GetStackDescription statusSrv;
  bool find = false;
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  for (auto t : statusSrv.response.tasks)
  {
    if (t.name == "new_go_to_position")
    {
      find = true;
      break;
    }
  }
  for (auto t : statusSrv.response.tasks)
  {
    if (t.name == "go_to_position")
    {
      find = false;
      break;
    }
  }
  EXPECT_TRUE(find);
}

TEST_F(WBCTests, GetTaskError2)
{
  ros::Duration(4.0).sleep();
  pal_wbc_msgs::GetTaskError srv;
  srv.request.id = "new_go_to_position";
  EXPECT_TRUE(getTaskErrorServ_.call(srv));
  EXPECT_LT(srv.response.taskError.error_norm, 1.e-2);
}

TEST_F(WBCTests, PushingOrientationTask)
{
  property_bag::PropertyBag taskProperties("taskType",
                                           std::string("pal_wbc/GoToOrientationMetaTask"));
  Eigen::Quaterniond target_orientation;

  target_orientation.w() = 1.0;
  target_orientation.x() = 0.0;
  target_orientation.y() = 0.0;
  target_orientation.z() = 0.0;

  taskProperties.addProperty("task_id", std::string("go_to_orientation"));
  taskProperties.addProperty("target_orientation", target_orientation);
  taskProperties.addProperty("tip_name", std::string("arm_tool_link"));
  taskProperties.addProperty("damping", 0.2);
  taskProperties.addProperty("signal_reference", reference_);

  pal_wbc_msgs::PushTask srv;
  srv.request.push_task_params.params = generateTaskDescription(taskProperties);
  srv.request.push_task_params.respect_task_id = "new_go_to_position";
  srv.request.push_task_params.order.order = pal_wbc_msgs::Order::After;
  srv.request.push_task_params.blend = blend_;

  EXPECT_TRUE(pushTaskServ_.call(srv));
}

TEST_F(WBCTests, GetStackDescription4)
{
  ros::Duration(1.0).sleep();
  pal_wbc_msgs::GetStackDescription statusSrv;
  bool find_orientation = false;
  bool find_position = false;
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  for (auto t : statusSrv.response.tasks)
  {
    if (t.name == "go_to_orientation")
    {
      find_orientation = true;
    }
    if(t.name == "new_go_to_position")
    {
      find_position = true;
    }
  }
  EXPECT_TRUE(find_orientation);
  EXPECT_TRUE(find_position);
}

TEST_F(WBCTests, GetTaskError3)
{
  ros::Duration(1.0).sleep();
  pal_wbc_msgs::GetTaskError srv;
  srv.request.id = "go_to_orientation";
  EXPECT_TRUE(getTaskErrorServ_.call(srv));
  EXPECT_LT(srv.response.taskError.error_norm, 1.e-2);
}

TEST_F(WBCTests, PushGazeTask)
{
      property_bag::PropertyBag taskProperties(
          "taskType", std::string("pal_wbc/GoToPointRayAngleGazeKinematicMetatask"));

      Eigen::Vector3d positionGoal(2, 0, 1.0);

      taskProperties.addProperty("task_id", std::string("gaze"));
      taskProperties.addProperty("target_position", positionGoal);
      taskProperties.addProperty("camera_frame", std::string("xtion_optical_frame"));
      taskProperties.addProperty("damping", 0.2);
      taskProperties.addProperty("reference_type", reference_);

      pal_wbc_msgs::PushTask srv;
      srv.request.push_task_params.params = generateTaskDescription(taskProperties);
      srv.request.push_task_params.respect_task_id = "go_to_orientation";
      srv.request.push_task_params.order.order = pal_wbc_msgs::Order::After;
      srv.request.push_task_params.blend = blend_;

      EXPECT_TRUE(pushTaskServ_.call(srv));
}

TEST_F(WBCTests, GetStackDescription5)
{
  ros::Duration(1.0).sleep();
  pal_wbc_msgs::GetStackDescription statusSrv;
  bool find_orientation = false;
  bool find_position = false;
  bool find_gaze = false;
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  for (auto t : statusSrv.response.tasks)
  {
    if (t.name == "go_to_orientation")
    {
      find_orientation = true;
    }
    if(t.name == "new_go_to_position")
    {
      find_position = true;
    }
    if(t.name == "gaze")
    {
      find_gaze = true;
    }
  }
  EXPECT_TRUE(find_orientation);
  EXPECT_TRUE(find_position);
  EXPECT_TRUE(find_gaze);
}

TEST_F(WBCTests, GetTaskError4)
{
  ros::Duration(1.0).sleep();
  pal_wbc_msgs::GetTaskError srv;
  srv.request.id = "gaze";
  EXPECT_TRUE(getTaskErrorServ_.call(srv));
  EXPECT_LT(srv.response.taskError.error_norm, 1.e-2);
}

TEST_F(WBCTests, PopGaze)
{
  pal_wbc_msgs::PopTask srv;
  srv.request.name = "gaze";
  srv.request.blend = blend_;
  EXPECT_TRUE(popTaskServ_.call(srv));
}

TEST_F(WBCTests, GetStackDescription6)
{
  ros::Duration(1.0).sleep();
  pal_wbc_msgs::GetStackDescription statusSrv;
  bool find_orientation = false;
  bool find_position = false;
  bool find_gaze = false;
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  for (auto t : statusSrv.response.tasks)
  {
    if (t.name == "go_to_orientation")
    {
      find_orientation = true;
    }
    if(t.name == "new_go_to_position")
    {
      find_position = true;
    }
    if(t.name == "gaze")
    {
      find_gaze = true;
    }
  }
  EXPECT_TRUE(find_orientation);
  EXPECT_TRUE(find_position);
  EXPECT_FALSE(find_gaze);
}

/*TEST_F(WBCTests, PopGaze2)
{
  pal_wbc_msgs::PopTask srv;
  srv.request.name = "gaze";
  srv.request.blend = blend_;
  EXPECT_FALSE(popTaskServ_.call(srv));
}*/

TEST_F(WBCTests, PopPosition)
{
  pal_wbc_msgs::PopTask srv;
  srv.request.name = "new_go_to_position";
  srv.request.blend = blend_;
  EXPECT_TRUE(popTaskServ_.call(srv));
}

TEST_F(WBCTests, GetStackDescription7)
{
  ros::Duration(1.0).sleep();
  pal_wbc_msgs::GetStackDescription statusSrv;
  bool find_orientation = false;
  bool find_position = false;
  bool find_gaze = false;
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  for (auto t : statusSrv.response.tasks)
  {
    if (t.name == "go_to_orientation")
    {
      find_orientation = true;
    }
    if(t.name == "new_go_to_position")
    {
      find_position = true;
    }
    if(t.name == "gaze")
    {
      find_gaze = true;
    }
  }
  EXPECT_TRUE(find_orientation);
  EXPECT_FALSE(find_position);
  EXPECT_FALSE(find_gaze);
}

TEST_F(WBCTests, PopOrientation)
{
  pal_wbc_msgs::PopTask srv;
  srv.request.name = "go_to_orientation";
  srv.request.blend = blend_;
  EXPECT_TRUE(popTaskServ_.call(srv));
}

TEST_F(WBCTests, GetStackDescription8)
{
  ros::Duration(1.0).sleep();
  pal_wbc_msgs::GetStackDescription statusSrv;
  bool find_orientation = false;
  bool find_position = false;
  bool find_gaze = false;
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  for (auto t : statusSrv.response.tasks)
  {
    if (t.name == "go_to_orientation")
    {
      find_orientation = true;
    }
    if(t.name == "new_go_to_position")
    {
      find_position = true;
    }
    if(t.name == "gaze")
    {
      find_gaze = true;
    }
  }
  EXPECT_FALSE(find_orientation);
  EXPECT_FALSE(find_position);
  EXPECT_FALSE(find_gaze);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "push_tasks_test");

  ros::NodeHandle nh;
  ros::Time::waitForValid();

  return RUN_ALL_TESTS();
}
