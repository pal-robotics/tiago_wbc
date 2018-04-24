#include <tiago_wbc/wbc_tests_fixture.h>
#include <pal_ros_utils/tf_utils.h>
#include <eigen_checks/gtest.h>

using namespace pal;

// Get initial stack description
TEST_F(WBCTests, GetStackDescription)
{
  pal_wbc_msgs::GetStackDescription statusSrv;
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
}

// Push a go to position task
TEST_F(WBCTests, PushingPositionTask)
{
  property_bag::PropertyBag taskProperties("taskType",
                                           std::string("pal_wbc/GoToPositionMetaTask"));

  taskProperties.addProperty("task_id", std::string("go_to_position"));
  taskProperties.addProperty("target_position", positionGoal_);
  taskProperties.addProperty("tip_name", tip_name_);
  taskProperties.addProperty("damping", 0.2);
  taskProperties.addProperty("signal_reference", reference_);
  taskProperties.addProperty("p_pos_gain", 4.0);

  pal_wbc_msgs::PushTask srv;
  srv.request.push_task_params.params = generateTaskDescription(taskProperties);
  srv.request.push_task_params.respect_task_id = "self_collision";
  srv.request.push_task_params.order.order = pal_wbc_msgs::Order::After;
  srv.request.push_task_params.blend = blend_;

  EXPECT_TRUE(pushTaskServ_.call(srv));
}

// Get the stack description with the go to position task
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


// Check that go to position task converges to the expected point
TEST_F(WBCTests, GetTaskError)
{
  ros::Duration(8.0).sleep();
  pal_wbc_msgs::GetTaskError srv;
  srv.request.id = "go_to_position";
  EXPECT_TRUE(getTaskErrorServ_.call(srv));
  EXPECT_LT(srv.response.taskError.error_norm, 1.e-4);

  eMatrixHom received_tf = getTransform(base_frame_, tip_name_, ros::Duration(2.0));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(positionGoal_, received_tf.translation(), 1.e-2));
}

// Replace the go to position task by a new go to position task
TEST_F(WBCTests, PushPositionTask2)
{
  property_bag::PropertyBag taskProperties("taskType",
                                           std::string("pal_wbc/GoToPositionMetaTask"));

  taskProperties.addProperty("task_id", std::string("new_go_to_position"));
  taskProperties.addProperty("target_position", positionGoal3_);
  taskProperties.addProperty("tip_name", std::string("arm_tool_link"));
  taskProperties.addProperty("damping", 0.2);
  taskProperties.addProperty("signal_reference", reference_);
  taskProperties.addProperty("p_pos_gain", 4.0);

  pal_wbc_msgs::PushPopTask srv;
  pal_wbc_msgs::PushTaskParams push_params;
  push_params.params = generateTaskDescription(taskProperties);
  push_params.respect_task_id = "go_to_position";
  push_params.order.order = pal_wbc_msgs::Order::Replace;
  push_params.blend = blend_;
  srv.request.push_tasks.push_back(push_params);
  EXPECT_TRUE(pushPopTaskServ_.call(srv));
}

// Check that this new go to position task replaces the previous one in the stack
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

// Check that the new go to position task converges to the expected point
TEST_F(WBCTests, GetTaskError2)
{
  ros::Duration(6.0).sleep();
  pal_wbc_msgs::GetTaskError srv;
  srv.request.id = "new_go_to_position";
  EXPECT_TRUE(getTaskErrorServ_.call(srv));
  EXPECT_LT(srv.response.taskError.error_norm, 1.e-4);

  eMatrixHom received_tf = getTransform(base_frame_, tip_name_, ros::Duration(2.0));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(positionGoal3_, received_tf.translation(), 1.e-2));
}

// Push an orientation task
TEST_F(WBCTests, PushingOrientationTask)
{
  property_bag::PropertyBag taskProperties("taskType",
                                           std::string("pal_wbc/GoToOrientationMetaTask"));

  taskProperties.addProperty("task_id", std::string("go_to_orientation"));
  taskProperties.addProperty("target_orientation", orientationGoal_);
  taskProperties.addProperty("tip_name", std::string("arm_tool_link"));
  taskProperties.addProperty("damping", 0.2);
  taskProperties.addProperty("signal_reference", reference_);
  taskProperties.addProperty("p_orient_gain", 4.0);

  pal_wbc_msgs::PushTask srv;
  srv.request.push_task_params.params = generateTaskDescription(taskProperties);
  srv.request.push_task_params.respect_task_id = "new_go_to_position";
  srv.request.push_task_params.order.order = pal_wbc_msgs::Order::After;
  srv.request.push_task_params.blend = blend_;

  EXPECT_TRUE(pushTaskServ_.call(srv));
}

// Get the stack description with the orientation task and the position task
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
    if (t.name == "new_go_to_position")
    {
      find_position = true;
    }
  }
  EXPECT_TRUE(find_orientation);
  EXPECT_TRUE(find_position);
}

// Check that the orientation task converges to the expected orientation
TEST_F(WBCTests, GetTaskError3)
{
  ros::Duration(10.0).sleep();
  pal_wbc_msgs::GetTaskError srv;
  srv.request.id = "go_to_orientation";
  EXPECT_TRUE(getTaskErrorServ_.call(srv));
  EXPECT_LT(srv.response.taskError.error_norm, 1.e-4);

  eMatrixHom received_tf = getTransform(base_frame_, tip_name_, ros::Duration(2.0));
  eVector3 quat_error = quaternion_error(orientationGoal_, eQuaternion(received_tf.rotation()));
  EXPECT_LT(quat_error.norm(), 1.e-2);
}

// Push a "new" gaze task
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

// Check that the "new" gaze task is in the stack description
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
    if (t.name == "new_go_to_position")
    {
      find_position = true;
    }
    if (t.name == "gaze")
    {
      find_gaze = true;
    }
  }
  EXPECT_TRUE(find_orientation);
  EXPECT_TRUE(find_position);
  EXPECT_TRUE(find_gaze);
}

// Check that the "new" gaze task converges
TEST_F(WBCTests, GetTaskError4)
{
  ros::Duration(1.0).sleep();
  pal_wbc_msgs::GetTaskError srv;
  srv.request.id = "gaze";
  EXPECT_TRUE(getTaskErrorServ_.call(srv));
  EXPECT_LT(srv.response.taskError.error_norm, 1.e-2);
}

// Pop the gaze task
TEST_F(WBCTests, PopGaze)
{
  pal_wbc_msgs::PopTask srv;
  srv.request.name = "gaze";
  srv.request.blend = blend_;
  EXPECT_TRUE(popTaskServ_.call(srv));
}

// Check that the gaze task doesn't appears anymore
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
    if (t.name == "new_go_to_position")
    {
      find_position = true;
    }
    if (t.name == "gaze")
    {
      find_gaze = true;
    }
  }
  EXPECT_TRUE(find_orientation);
  EXPECT_TRUE(find_position);
  EXPECT_FALSE(find_gaze);
}

// Pop again the gaze tasks
// @todo the service should output an error
TEST_F(WBCTests, PopGaze2)
{
  pal_wbc_msgs::PopTask srv;
  srv.request.name = "gaze";
  srv.request.blend = blend_;
  EXPECT_TRUE(popTaskServ_.call(srv));
}

// Pop the go to position task
TEST_F(WBCTests, PopPosition)
{
  pal_wbc_msgs::PopTask srv;
  srv.request.name = "new_go_to_position";
  srv.request.blend = blend_;
  EXPECT_TRUE(popTaskServ_.call(srv));
}

// Check that the go to position task doesn't appers in the stack
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
    if (t.name == "new_go_to_position")
    {
      find_position = true;
    }
    if (t.name == "gaze")
    {
      find_gaze = true;
    }
  }
  EXPECT_TRUE(find_orientation);
  EXPECT_FALSE(find_position);
  EXPECT_FALSE(find_gaze);
}

// Pop the orientation task
TEST_F(WBCTests, PopOrientation)
{
  pal_wbc_msgs::PopTask srv;
  srv.request.name = "go_to_orientation";
  srv.request.blend = blend_;
  EXPECT_TRUE(popTaskServ_.call(srv));
}

// Check taht the go to orientation task doesn't appears in the stack
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
    if (t.name == "new_go_to_position")
    {
      find_position = true;
    }
    if (t.name == "gaze")
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
