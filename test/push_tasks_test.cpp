#include "wbc_tests_fixture.h"
#include <pal_ros_utils/tf_utils.h>
#include <eigen_checks/gtest.h>
#include <algorithm>
#include <sensor_msgs/JointState.h>
#include <pal_utils/exception_utils.h>

using namespace pal;

TEST_F(WBCTests, PushTasksTest)
{
  // Get initial stack description
  pal_wbc_msgs::GetStackDescription statusSrv;
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));

  // Push a go to position task
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

  // Get the stack description with the go to position task
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  EXPECT_TRUE(stack_description_contains("go_to_position"));

  // Check that go to position task converges to the expected point
  ros::Duration(8.0).sleep();
  pal_wbc_msgs::GetTaskError errorSrv;
  errorSrv.request.id = "go_to_position";
  EXPECT_TRUE(getTaskErrorServ_.call(errorSrv));
  EXPECT_LT(errorSrv.response.taskError.error_norm, 1.e-2);

  eMatrixHom received_tf = getTransform(base_frame_, tip_name_, ros::Duration(2.0));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(positionGoal_, received_tf.translation(), 1.e-2));

  // Replace the go to position task by a new go to position task
  taskProperties.updateProperty("task_id", std::string("new_go_to_position"));
  taskProperties.updateProperty("target_position", positionGoal3_);

  pal_wbc_msgs::PushTask replace_srv;
  pal_wbc_msgs::PushTaskParams push_params;
  push_params.params = generateTaskDescription(taskProperties);
  push_params.respect_task_id = "go_to_position";
  push_params.order.order = pal_wbc_msgs::Order::Replace;
  push_params.blend = blend_;
  replace_srv.request.push_task_params = push_params;
  EXPECT_TRUE(pushTaskServ_.call(replace_srv));

  // Check that this new go to position task replaces the previous one in the stack
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  EXPECT_TRUE(stack_description_contains("new_go_to_position"));
  EXPECT_FALSE(stack_description_contains("go_to_position"));

  // Check that the new go to position task converges to the expected point
  ros::Duration(10.0).sleep();
  errorSrv.request.id = "new_go_to_position";
  EXPECT_TRUE(getTaskErrorServ_.call(errorSrv));
  EXPECT_LT(errorSrv.response.taskError.error_norm, 1.e-2);

  eMatrixHom received_tf_2 = getTransform(base_frame_, tip_name_, ros::Duration(2.0));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(positionGoal3_, received_tf_2.translation(), 1.e-2));

  // Push an orientation task
  property_bag::PropertyBag orientation_taskProperties(
      "taskType", std::string("pal_wbc/GoToOrientationMetaTask"));

  orientation_taskProperties.addProperty("task_id", std::string("go_to_orientation"));
  orientation_taskProperties.addProperty("target_orientation", orientationGoal_);
  orientation_taskProperties.addProperty("tip_name", std::string("arm_tool_link"));
  orientation_taskProperties.addProperty("damping", 0.2);
  orientation_taskProperties.addProperty("signal_reference", reference_);
  orientation_taskProperties.addProperty("p_orient_gain", 4.0);

  srv.request.push_task_params.params = generateTaskDescription(orientation_taskProperties);
  srv.request.push_task_params.respect_task_id = "new_go_to_position";
  srv.request.push_task_params.order.order = pal_wbc_msgs::Order::After;
  srv.request.push_task_params.blend = blend_;

  EXPECT_TRUE(pushTaskServ_.call(srv));

  // Get the stack description with the orientation task and the position task
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  EXPECT_TRUE(stack_description_contains("new_go_to_position"));
  EXPECT_TRUE(stack_description_contains("go_to_orientation"));

  // Check that the orientation task converges to the expected orientation
  ros::Duration(10.0).sleep();
  errorSrv.request.id = "go_to_orientation";
  EXPECT_TRUE(getTaskErrorServ_.call(errorSrv));
  EXPECT_LT(errorSrv.response.taskError.error_norm, 1.e-2);

  eMatrixHom received_tf_3 = getTransform(base_frame_, tip_name_, ros::Duration(2.0));
  eVector3 quat_error =
      quaternion_error(orientationGoal_, eQuaternion(received_tf_3.rotation()));
  EXPECT_LT(quat_error.norm(), 1.e-2);

  // Push a "new" gaze task
  property_bag::PropertyBag gaze_taskProperties(
      "taskType", std::string("pal_wbc/GoToPointRayAngleGazeKinematicMetatask"));

  Eigen::Vector3d positionGoal(2, 0, 1.0);

  gaze_taskProperties.addProperty("task_id", std::string("gaze"));
  gaze_taskProperties.addProperty("target_position", positionGoal);
  gaze_taskProperties.addProperty("camera_frame", std::string("xtion_optical_frame"));
  gaze_taskProperties.addProperty("damping", 0.2);
  gaze_taskProperties.addProperty("reference_type", reference_);

  srv.request.push_task_params.params = generateTaskDescription(gaze_taskProperties);
  srv.request.push_task_params.respect_task_id = "go_to_orientation";
  srv.request.push_task_params.order.order = pal_wbc_msgs::Order::After;
  srv.request.push_task_params.blend = blend_;

  EXPECT_TRUE(pushTaskServ_.call(srv));

  // Check that the "new" gaze task is in the stack description
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  EXPECT_TRUE(stack_description_contains("new_go_to_position"));
  EXPECT_TRUE(stack_description_contains("go_to_orientation"));
  EXPECT_TRUE(stack_description_contains("gaze"));

  // Check that the "new" gaze task converges
  ros::Duration(1.0).sleep();
  errorSrv.request.id = "gaze";
  EXPECT_TRUE(getTaskErrorServ_.call(errorSrv));
  EXPECT_LT(errorSrv.response.taskError.error_norm, 1.e-2);

  // Pop the gaze task
  pal_wbc_msgs::PopTask pop_srv;
  pop_srv.request.name = "gaze";
  pop_srv.request.blend = blend_;
  EXPECT_TRUE(popTaskServ_.call(pop_srv));

  // Check that the gaze task doesn't appears anymore
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  EXPECT_TRUE(stack_description_contains("new_go_to_position"));
  EXPECT_TRUE(stack_description_contains("go_to_orientation"));
  EXPECT_FALSE(stack_description_contains("gaze"));

  // Pop again the gaze tasks
  // @todo the service should output an error
  pop_srv.request.name = "gaze";
  pop_srv.request.blend = blend_;
  EXPECT_TRUE(popTaskServ_.call(pop_srv));

  // Pop the go to position task
  pop_srv.request.name = "new_go_to_position";
  pop_srv.request.blend = blend_;
  EXPECT_TRUE(popTaskServ_.call(pop_srv));

  // Check that the go to position task doesn't appers in the stack
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  EXPECT_FALSE(stack_description_contains("new_go_to_position"));
  EXPECT_TRUE(stack_description_contains("go_to_orientation"));
  EXPECT_FALSE(stack_description_contains("gaze"));

  // Pop the orientation task
  pop_srv.request.name = "go_to_orientation";
  pop_srv.request.blend = blend_;
  EXPECT_TRUE(popTaskServ_.call(pop_srv));

  // Check that the go to orientation task doesn't appears in the stack
  ros::Duration(1.0).sleep();
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  EXPECT_FALSE(stack_description_contains("new_go_to_position"));
  EXPECT_FALSE(stack_description_contains("go_to_orientation"));
  EXPECT_FALSE(stack_description_contains("gaze"));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "push_tasks_test");

  ros::NodeHandle nh;
  ros::Time::waitForValid();

  {
    boost::shared_ptr<const sensor_msgs::JointState> joint_state_msg;
    // AS: Wait for the whole body controller to come up. It would be nice to
    // find a way to avoid hardcoding controller name
    joint_state_msg = ros::topic::waitForMessage<sensor_msgs::JointState>(
        "/whole_body_kinematic_controller/joint_states", nh, ros::Duration(300.0));
    PAL_ASSERT_PERSIST(NULL != joint_state_msg, "Controller is not running.");
  }

  return RUN_ALL_TESTS();
}
