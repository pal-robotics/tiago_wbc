#include <tiago_wbc/wbc_tests_fixture.h>
#include <pal_ros_utils/tf_utils.h>
#include <eigen_checks/gtest.h>
#include <smach_c/state_machine_with_introspection.h>
#include <smach_c_wbc_states/move_tip_to_pose_wbc_state.h>
#include <smach_c_wbc_states/pop_tasks_state.h>
#include <smach_c_wbc_states/gaze_point_wbc_state.h>

using namespace pal;

// Push a move tip to desired position task and check that converges to the given point
TEST_F(WBCTests, MoveTiptoDesiredPositionTask)
{
  ros::NodeHandle nh("~");
  smach_c::UserData user_data;

  smach_c::StateMachineWithIntrospectionPtr sm(new smach_c::StateMachineWithIntrospection(
      nh, { smach_c::SUCCESS, smach_c::FAILURE, smach_c::PREEMPTED }));

  geometry_msgs::PointStamped target_position;
  pal::convert(positionGoal_, target_position.point);
  target_position.header.frame_id = base_frame_;

  pal::MoveTiptoDesiredPositionPtr move_tip_position(new pal::MoveTiptoDesiredPosition(
      target_position, {}, std::string("test_position"), {}, tip_name_, pal_wbc_msgs::Order::After,
      std::string("self_collision"), double(1.e-4), ros::Duration(10.0)));


  sm->add("Move tip position", move_tip_position,
          { { smach_c::SUCCESS, smach_c::SUCCESS },
            { smach_c::FAILURE, smach_c::FAILURE },
            { smach_c::PREEMPTED, smach_c::PREEMPTED } });

  EXPECT_EQ(smach_c::SUCCESS, sm->execute(user_data));

  pal_wbc_msgs::GetStackDescription statusSrv;
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  EXPECT_TRUE(stack_description_contains("test_position"));

  pal_wbc_msgs::GetTaskError errorSrv;
  errorSrv.request.id = "test_position";
  EXPECT_TRUE(getTaskErrorServ_.call(errorSrv));
  EXPECT_LT(errorSrv.response.taskError.error_norm, 1.e-4);

  eMatrixHom received_tf = getTransform(base_frame_, tip_name_, ros::Duration(2.0));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(positionGoal_, received_tf.translation(), 1.e-2));

  // Replace the move tip to desired position task with a new one whithout any target pose
  // and with a tip offset and check that converges to the given point
  sm.reset(new smach_c::StateMachineWithIntrospection(
      nh, { smach_c::SUCCESS, smach_c::FAILURE, smach_c::PREEMPTED }));

  geometry_msgs::Pose tip_offset;
  eVector3 tip_offset_vec(0.1, -0.15, 0.2);

  tip_offset.position.x = tip_offset_vec[0];
  tip_offset.position.y = tip_offset_vec[1];
  tip_offset.position.z = tip_offset_vec[2];

  move_tip_position.reset(new pal::MoveTiptoDesiredPosition(
      {}, {}, std::string("test_position_offset"), tip_offset, tip_name_,
      pal_wbc_msgs::Order::Replace, std::string("test_position"), double(1.e-4),
      ros::Duration(10.0)));


  sm->add("Move tip position", move_tip_position,
          { { smach_c::SUCCESS, smach_c::SUCCESS },
            { smach_c::FAILURE, smach_c::FAILURE },
            { smach_c::PREEMPTED, smach_c::PREEMPTED } });

  EXPECT_EQ(smach_c::SUCCESS, sm->execute(user_data));

  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  EXPECT_TRUE(stack_description_contains("test_position_offset"));
  EXPECT_FALSE(stack_description_contains("test_position"));

  errorSrv.request.id = "test_position_offset";
  EXPECT_TRUE(getTaskErrorServ_.call(errorSrv));
  EXPECT_LT(errorSrv.response.taskError.error_norm, 1.e-4);

  eMatrixHom received_tf_2 = getTransform(base_frame_, tip_name_, ros::Duration(2.0));
  tip_offset_vec = received_tf_2.rotation() * tip_offset_vec;

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(positionGoal_,
                                received_tf_2.translation() + tip_offset_vec, 1.e-2));

  // Use PopTasksState to pop the given move tip to desired position
  sm.reset(new smach_c::StateMachineWithIntrospection(
      nh, { smach_c::SUCCESS, smach_c::FAILURE, smach_c::PREEMPTED }));

  std::vector<std::string> tasks;
  tasks.push_back("test_position_offset");
  smach_c::StatePtr pop_state(new pal::PopTasksState(tasks));

  sm->add("Pop state", pop_state,
          { { smach_c::SUCCESS, smach_c::SUCCESS },
            { smach_c::FAILURE, smach_c::FAILURE },
            { smach_c::PREEMPTED, smach_c::PREEMPTED } });

  EXPECT_EQ(smach_c::SUCCESS, sm->execute(user_data));

  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  EXPECT_FALSE(stack_description_contains("test_position_offset"));

// Use PopTasksState to pop again the given move tip to desired position
// @todo this state should return failure
  EXPECT_EQ(smach_c::SUCCESS, sm->execute(user_data));
}

// Push a gaze task state and check that converges
TEST_F(WBCTests, GazeTask)
{
  ros::NodeHandle nh("~");
  smach_c::UserData user_data;

  smach_c::StateMachineWithIntrospectionPtr sm(new smach_c::StateMachineWithIntrospection(
      nh, { smach_c::SUCCESS, smach_c::FAILURE, smach_c::PREEMPTED }));

  geometry_msgs::PointStamped target_position;
  target_position.header.frame_id = base_frame_;

  target_position.point.x = 2.0;
  target_position.point.y = -2.0;
  target_position.point.z = 2.0;

  pal::GazePointWBCStatePtr gaze_task(new pal::GazePointWBCState(
      target_position, std::string("xtion_optical_frame"), std::string("gaze_task"),
      pal_wbc_msgs::Order::After, std::string("self_collision")));

  sm->add("Gaze task", gaze_task,
          { { smach_c::SUCCESS, smach_c::SUCCESS },
            { smach_c::FAILURE, smach_c::FAILURE },
            { smach_c::PREEMPTED, smach_c::PREEMPTED } });

  EXPECT_EQ(smach_c::SUCCESS, sm->execute(user_data));

  pal_wbc_msgs::GetStackDescription statusSrv;
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  EXPECT_TRUE(stack_description_contains("gaze_task"));
}

// Push a move tip to desired pose state
TEST_F(WBCTests, MoveTiptoDesiredPoseTask)
{
  ros::NodeHandle nh("~");
  smach_c::UserData user_data;

  smach_c::StateMachineWithIntrospectionPtr sm(new smach_c::StateMachineWithIntrospection(
      nh, { smach_c::SUCCESS, smach_c::FAILURE, smach_c::PREEMPTED }));

  geometry_msgs::PoseStamped target_pose;
  pal::convert(positionGoal2_, target_pose.pose.position);
  pal::convert(orientationGoal_, target_pose.pose.orientation);
  target_pose.header.frame_id = base_frame_;

  MoveTiptoDesiredPosePtr move_tip_pose(new MoveTiptoDesiredPose(
      target_pose, {}, std::string("test_pose"), {}, tip_name_, pal_wbc_msgs::Order::Before,
      std::string("default_reference"), double(1.e-4), ros::Duration(20.0)));

  sm->add("Move tip pose", move_tip_pose,
          { { smach_c::SUCCESS, smach_c::SUCCESS },
            { smach_c::FAILURE, smach_c::FAILURE },
            { smach_c::PREEMPTED, smach_c::PREEMPTED } });

  EXPECT_EQ(smach_c::SUCCESS, sm->execute(user_data));

  pal_wbc_msgs::GetStackDescription statusSrv;
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  EXPECT_TRUE(stack_description_contains("test_pose"));

  pal_wbc_msgs::GetTaskError errorSrv;
  errorSrv.request.id = "test_pose";
  EXPECT_TRUE(getTaskErrorServ_.call(errorSrv));
  EXPECT_LT(errorSrv.response.taskError.error_norm, 1.e-4);

  eMatrixHom received_tf = getTransform(base_frame_, tip_name_, ros::Duration(2.0));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(positionGoal2_, received_tf.translation(), 1.e-2));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(orientationGoal_.toRotationMatrix(),
                                received_tf.rotation(), 1.e-2));

// Replace the move tip to desired pose state with a new target pose with a given offset
// in position and orientation and checks that it converges to the expected pose.
sm.reset(new smach_c::StateMachineWithIntrospection(
      nh, { smach_c::SUCCESS, smach_c::FAILURE, smach_c::PREEMPTED }));

  pal::convert(positionGoal_, target_pose.pose.position);

  geometry_msgs::Pose tip_offset;
  pal::convert(orientationGoal2_, tip_offset.orientation);
  eVector3 tip_offset_vec(0.1, -0.15, 0.2);
  tip_offset.position.x = tip_offset_vec[0];
  tip_offset.position.y = tip_offset_vec[1];
  tip_offset.position.z = tip_offset_vec[2];

  move_tip_pose.reset(
      new MoveTiptoDesiredPose(target_pose, {}, std::string("test_pose_offset"),
                               tip_offset, tip_name_, pal_wbc_msgs::Order::Replace,
                               std::string("test_pose"), double(1.e-4), ros::Duration(30.0)));

  sm->add("Move tip pose", move_tip_pose,
          { { smach_c::SUCCESS, smach_c::SUCCESS },
            { smach_c::FAILURE, smach_c::FAILURE },
            { smach_c::PREEMPTED, smach_c::PREEMPTED } });

  EXPECT_EQ(smach_c::SUCCESS, sm->execute(user_data));

  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  EXPECT_TRUE(stack_description_contains("test_pose_offset"));

  errorSrv.request.id = "test_pose_offset";
  EXPECT_TRUE(getTaskErrorServ_.call(errorSrv));
  EXPECT_LT(errorSrv.response.taskError.error_norm, 1.e-4);

  eMatrixHom received_tf_2 = getTransform(base_frame_, tip_name_, ros::Duration(2.0));
  received_tf_2 = received_tf_2 * createMatrix(orientationGoal2_, tip_offset_vec);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(positionGoal_, received_tf_2.translation(), 1.e-2));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(orientationGoal_.toRotationMatrix(),
                                received_tf_2.rotation(), 1.e-2));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "move_tip_smach_c_test");

  ros::NodeHandle nh;
  ros::Time::waitForValid();

  return RUN_ALL_TESTS();
}
