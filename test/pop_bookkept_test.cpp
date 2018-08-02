#include "wbc_tests_fixture.h"
#include <eigen_checks/gtest.h>
#include <pal_ros_utils/conversions.h>
#include <smach_c/state_machine_with_introspection.h>
#include <smach_c_wbc_states/move_tip_to_pose_wbc_state.h>
#include <smach_c_wbc_states/gaze_point_wbc_state.h>
#include <smach_c_wbc_states/pop_bookkept_tasks_state.h>
#include <sensor_msgs/JointState.h>
#include <pal_utils/exception_utils.h>

using namespace pal;

// Replace the gaze and the move tip to pose by new states and check that
// PopBookkeptTasksState pop all of them
TEST_F(WBCTests, PopBookkeptTasks)
{
  ros::NodeHandle nh("~");
  smach_c::UserData user_data;
  std::string wbc_bookkeeping_property = "wbc_bookkeeping";

  smach_c::StateMachineWithIntrospectionPtr sm(new smach_c::StateMachineWithIntrospection(
      nh, { smach_c::SUCCESS, smach_c::FAILURE, smach_c::PREEMPTED }));

  geometry_msgs::PoseStamped target_pose;
  pal::convert(positionGoal_, target_pose.pose.position);
  pal::convert(orientationGoal_, target_pose.pose.orientation);
  target_pose.header.frame_id = base_frame_;

  MoveTiptoDesiredPosePtr move_tip_pose(new MoveTiptoDesiredPose(
      target_pose, {}, std::string("test_pose_offset"), {}, tip_name_, pal_wbc_msgs::Order::After,
      std::string("self_collision"), double(1.e-4), ros::Duration(20.0)));

  sm->add("Move tip pose", move_tip_pose,
          { { smach_c::SUCCESS, "Gaze task" },
            { smach_c::FAILURE, smach_c::FAILURE },
            { smach_c::PREEMPTED, smach_c::PREEMPTED } },
          { { WBCTaskState::WBC_BOOKKEEPING_UD, wbc_bookkeeping_property } });

  geometry_msgs::PointStamped target_position;
  target_position.header.frame_id = "base_link";

  target_position.point.x = 2.0;
  target_position.point.y = 0.0;
  target_position.point.z = 0.5;

  pal::GazePointWBCStatePtr gaze_task(new pal::GazePointWBCState(
      target_position, std::string("xtion_optical_frame"), std::string("gaze_task"),
      pal_wbc_msgs::Order::Before, std::string("test_pose_offset")));

  sm->add("Gaze task", gaze_task,
          { { smach_c::SUCCESS, "Replace move tip pose" },
            { smach_c::FAILURE, smach_c::FAILURE },
            { smach_c::PREEMPTED, smach_c::PREEMPTED } },
          { { WBCTaskState::WBC_BOOKKEEPING_UD, wbc_bookkeeping_property } });

  MoveTiptoDesiredPosePtr second_move_tip_pose(new MoveTiptoDesiredPose(
      target_pose, {}, std::string("test_pose_offset"), {}, tip_name_, pal_wbc_msgs::Order::Replace,
      std::string("test_pose_offset"), double(1.e-4), ros::Duration(20.0)));

  sm->add("Replace move tip pose", second_move_tip_pose,
          { { smach_c::SUCCESS, "Replace gaze task" },
            { smach_c::FAILURE, smach_c::FAILURE },
            { smach_c::PREEMPTED, smach_c::PREEMPTED } },
          { { WBCTaskState::WBC_BOOKKEEPING_UD, wbc_bookkeeping_property } });

  pal::GazePointWBCStatePtr second_gaze_task(new pal::GazePointWBCState(
      target_position, std::string("xtion_optical_frame"), std::string("gaze_task"),
      pal_wbc_msgs::Order::Replace, std::string("gaze_task")));

  sm->add("Replace gaze task", second_gaze_task,
          { { smach_c::SUCCESS, "Pop states" },
            { smach_c::FAILURE, smach_c::FAILURE },
            { smach_c::PREEMPTED, smach_c::PREEMPTED } },
          { { WBCTaskState::WBC_BOOKKEEPING_UD, wbc_bookkeeping_property } });

  PopBookkeptTasksStatePtr pop_states(new PopBookkeptTasksState());

  sm->add("Pop states", pop_states,
          { { smach_c::SUCCESS, smach_c::SUCCESS },
            { smach_c::FAILURE, smach_c::FAILURE },
            { smach_c::PREEMPTED, smach_c::PREEMPTED } },
          { { WBCTaskState::WBC_BOOKKEEPING_UD, wbc_bookkeeping_property } });

  EXPECT_EQ(smach_c::SUCCESS, sm->execute(user_data));

  pal_wbc_msgs::GetStackDescription statusSrv;
  EXPECT_TRUE(stackDescriptionServ_.call(statusSrv));
  EXPECT_FALSE(stack_description_contains("test_pose_offset"));
  EXPECT_FALSE(stack_description_contains("gaze_task"));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "pop_bookkept_test");

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
