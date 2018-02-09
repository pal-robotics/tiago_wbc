#include <pal_wbc_utils/pal_wbc_utils.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "push_interactive_marker");
  ros::NodeHandle nh;

  pal::WBCServiceHelper srv_helper(nh);

  std::string reference_type;
  nh.param<std::string>("source_data", reference_type, "interactive_marker");

  std::string tip_name;
  nh.param<std::string>("tip_name", tip_name, "arm_7_link");

  std::string camera_frame;
  nh.param<std::string>("camera_frame", camera_frame, "xtion_optical_frame");

  property_bag::PropertyBag task;
  pal_wbc_msgs::Order::_order_type order = pal_wbc_msgs::Order::After;

  task.addProperty("taskType", std::string("pal_wbc/GoToPositionMetaTask"));
  task.addProperty("task_id",
                   std::string("interactive_marker_position_" + tip_name));
  task.addProperty("signal_reference", reference_type);
  task.addProperty("tip_name", tip_name);
  task.addProperty("damping", 0.2);
  task.addProperty("weight", 50.0);

  srv_helper.pushTask(task,
                      std::string("interactive_marker_position_" + tip_name),
                      order, "self_collision");

  task.updateProperty("taskType",
                      std::string("pal_wbc/GoToOrientationMetaTask"));
  task.updateProperty(
      "task_id", std::string("interactive_marker_orientation_" + tip_name));

  srv_helper.pushTask(
      task, std::string("interactive_marker_orientation_" + tip_name), order,
      std::string("interactive_marker_position_" + tip_name));

  property_bag::PropertyBag gaze_task;
  gaze_task.addProperty(
      "taskType",
      std::string("pal_wbc/GoToPointRayAngleGazeKinematicMetatask"));
  gaze_task.addProperty("task_id", std::string("gaze_task"));
  gaze_task.addProperty("reference_type", reference_type);
  gaze_task.addProperty("camera_frame", camera_frame);

  srv_helper.pushTask(gaze_task, "gaze_task", order,
                      std::string("interactive_marker_position_" + tip_name));

  ros::spin();
}
