#include <iostream>
#include <ros/ros.h>
#include <pal_wbc_msgs/TaskDescription.h>
#include <pal_wbc_msgs/GetStackDescription.h>
#include <pal_wbc_msgs/GetTaskError.h>
#include <pal_wbc_msgs/PopTask.h>
#include <pal_wbc_msgs/PushTask.h>
#include <pal_wbc_msgs/PushPopTask.h>
#include <Eigen/Dense>
#include <property_bag/property_bag.h>
#include <property_bag/serialization/property_bag_boost_serialization.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

std::string generateTaskDescription(const property_bag::PropertyBag properties){

  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  oa << properties;
  return ss.str();
}

int main(int argc, char** argv){

  ros::init(argc, argv, "wbc_reemc_hardware_test");
  ros::NodeHandle nh("~");

  ros::ServiceClient stackDescriptionServ = nh.serviceClient<pal_wbc_msgs::GetStackDescription>("/whole_body_kinematic_controller/get_stack_description");
  ros::ServiceClient popTaskServ = nh.serviceClient<pal_wbc_msgs::PopTask>("/whole_body_kinematic_controller/pop_task");
  ros::ServiceClient pushTaskServ = nh.serviceClient<pal_wbc_msgs::PushTask>("/whole_body_kinematic_controller/push_task");
  ros::ServiceClient pushPopTaskServ = nh.serviceClient<pal_wbc_msgs::PushPopTask>("/whole_body_kinematic_controller/push_pop_task");
  ros::ServiceClient getTaskErrorServ = nh.serviceClient<pal_wbc_msgs::GetTaskError>("/whole_body_kinematic_controller/get_task_error");

  //  std::string signal_type = "pointer_reflexx_typeII";
  std::string signal_type = "pointer";
  bool blend = true;

  {
    //Get current stack description
    pal_wbc_msgs::GetStackDescription statusSrv;
    if (stackDescriptionServ.call(statusSrv)){
      ROS_INFO_STREAM("Stack description:");
      ROS_INFO_STREAM(statusSrv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }

  }

  {
    //Push the goto position right arm task
    ROS_INFO_STREAM("Pushing new gotoPosition task");
    property_bag::PropertyBag taskProperties("taskType", std::string("pal_wbc/GoToPositionMetaTask"));
    Eigen::Vector3d positionGoal(0.54637, -0.13707, 0.95628);

    taskProperties.addProperty("task_id", std::string("go_to_position"));
    taskProperties.addProperty("target_position", positionGoal);
    taskProperties.addProperty("tip_name", std::string("arm_tool_link"));
    taskProperties.addProperty("damping", 0.2);
    taskProperties.addProperty("signal_reference", signal_type);

    pal_wbc_msgs::PushTask srv;
    srv.request.push_task_params.params = generateTaskDescription(taskProperties);
    srv.request.push_task_params.respect_task_id = "self_collision";
    srv.request.push_task_params.order.order = pal_wbc_msgs::Order::After;
    srv.request.push_task_params.blend = blend;
    if (pushTaskServ.call(srv)){
      ROS_INFO_STREAM("Succesfully pushed task:");
      ROS_INFO_STREAM(srv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }
  }

  {
    //Get current stack description
    pal_wbc_msgs::GetStackDescription statusSrv;
    if (stackDescriptionServ.call(statusSrv)){
      ROS_INFO_STREAM("Stack description:");
      ROS_INFO_STREAM(statusSrv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }

  }


  {
    ros::Duration(0.1).sleep();
    pal_wbc_msgs::GetTaskError srv;
    srv.request.id = "go_to_position";
    getTaskErrorServ.call(srv);
    ROS_INFO_STREAM("Task: "<<srv.request.id<<" error norm is: "<<srv.response.taskError.error_norm);
    int cont = 0;
    while(srv.response.taskError.error_norm > 1e-2){
      ROS_INFO_STREAM("Task: "<<srv.request.id<<" error norm is: "<<srv.response.taskError.error_norm);
      ros::Duration(0.1).sleep();
      getTaskErrorServ.call(srv);

      if(cont > 100){
        ROS_ERROR_STREAM("Task did not converge");
        break;
      }
      else{
        ++cont;
      }
    }

  }

  {
    //Get current stack description
    pal_wbc_msgs::GetStackDescription statusSrv;
    if (stackDescriptionServ.call(statusSrv)){
      ROS_INFO_STREAM("Stack description:");
      ROS_INFO_STREAM(statusSrv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }

  }

  {
    //Push the goto position right arm task
    ROS_INFO_STREAM("Pushing new gotoPosition task");
    property_bag::PropertyBag taskProperties("taskType", std::string("pal_wbc/GoToPositionMetaTask"));
    Eigen::Vector3d positionGoal(0.74637, -0.13707, 0.95628);

    taskProperties.addProperty("task_id", std::string("go_to_position"));
    taskProperties.addProperty("target_position", positionGoal);
    taskProperties.addProperty("tip_name", std::string("arm_tool_link"));
    taskProperties.addProperty("damping", 0.2);
    taskProperties.addProperty("signal_reference", signal_type);

    pal_wbc_msgs::PushPopTask srv;
    pal_wbc_msgs::PushTaskParams push_params;
    push_params.params = generateTaskDescription(taskProperties);
    push_params.respect_task_id = "go_to_position";
    push_params.order.order = pal_wbc_msgs::Order::Replace;
    push_params.blend = blend;
    srv.request.push_tasks.push_back(push_params);
    if (pushPopTaskServ.call(srv)){
      ROS_INFO_STREAM("Succesfully pushed task:");
      ROS_INFO_STREAM(srv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }
  }

  {
    //Get current stack description
    pal_wbc_msgs::GetStackDescription statusSrv;
    if (stackDescriptionServ.call(statusSrv)){
      ROS_INFO_STREAM("Stack description:");
      ROS_INFO_STREAM(statusSrv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }

  }

  {
    ros::Duration(0.1).sleep();
    pal_wbc_msgs::GetTaskError srv;
    srv.request.id = "go_to_position";
    getTaskErrorServ.call(srv);
    ROS_INFO_STREAM("Task: "<<srv.request.id<<" error norm is: "<<srv.response.taskError.error_norm);
    int cont = 0;
    while(srv.response.taskError.error_norm > 1e-2){
      ROS_INFO_STREAM("Task: "<<srv.request.id<<" error norm is: "<<srv.response.taskError.error_norm);
      ros::Duration(0.1).sleep();
      getTaskErrorServ.call(srv);

      if(cont > 100){
        ROS_ERROR_STREAM("Task did not converge");
        break;
      }
      else{
        ++cont;
      }
    }

  }

  {
    //Get current stack description
    pal_wbc_msgs::GetStackDescription statusSrv;
    if (stackDescriptionServ.call(statusSrv)){
      ROS_INFO_STREAM("Stack description:");
      ROS_INFO_STREAM(statusSrv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }

  }

  {
    //Push the goto position right arm task
    ROS_INFO_STREAM("Pushing new gotoPosition task");
    property_bag::PropertyBag taskProperties("taskType", std::string("pal_wbc/GoToPositionMetaTask"));
    Eigen::Vector3d positionGoal(0.54637, -0.13707, 0.95628);

    taskProperties.addProperty("task_id", std::string("go_to_position"));
    taskProperties.addProperty("target_position", positionGoal);
    taskProperties.addProperty("tip_name", std::string("arm_tool_link"));
    taskProperties.addProperty("damping", 0.2);
    taskProperties.addProperty("signal_reference", signal_type);

    pal_wbc_msgs::PushPopTask srv;
    pal_wbc_msgs::PushTaskParams push_params;
    push_params.params = generateTaskDescription(taskProperties);
    push_params.respect_task_id = "go_to_position";
    push_params.order.order = pal_wbc_msgs::Order::Replace;
    push_params.blend = blend;
    srv.request.push_tasks.push_back(push_params);
    if (pushPopTaskServ.call(srv)){
      ROS_INFO_STREAM("Succesfully pushed task:");
      ROS_INFO_STREAM(srv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }
  }

  {
    //Get current stack description
    pal_wbc_msgs::GetStackDescription statusSrv;
    if (stackDescriptionServ.call(statusSrv)){
      ROS_INFO_STREAM("Stack description:");
      ROS_INFO_STREAM(statusSrv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }

  }

  {
    ros::Duration(0.1).sleep();
    pal_wbc_msgs::GetTaskError srv;
    srv.request.id = "go_to_position";
    getTaskErrorServ.call(srv);
    ROS_INFO_STREAM("Task: "<<srv.request.id<<" error norm is: "<<srv.response.taskError.error_norm);
    int cont = 0;
    while(srv.response.taskError.error_norm > 1e-2){
      ROS_INFO_STREAM("Task: "<<srv.request.id<<" error norm is: "<<srv.response.taskError.error_norm);
      ros::Duration(0.1).sleep();
      getTaskErrorServ.call(srv);

      if(cont > 100){
        ROS_ERROR_STREAM("Task did not converge");
        break;
      }
      else{
        ++cont;
      }
    }

  }

  {
    //Get current stack description
    pal_wbc_msgs::GetStackDescription statusSrv;
    if (stackDescriptionServ.call(statusSrv)){
      ROS_INFO_STREAM("Stack description:");
      ROS_INFO_STREAM(statusSrv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }

  }

  {
    //Push the goto orientation task right arm task
    ROS_INFO_STREAM("Pushing new GotoOrientation task");
    property_bag::PropertyBag taskProperties("taskType", std::string("pal_wbc/GoToOrientationMetaTask"));
    Eigen::Quaterniond target_orientation;

    target_orientation.w() = 1.0;
    target_orientation.x() = 0.0;
    target_orientation.y() =  0.0;
    target_orientation.z() = 0.0;

    taskProperties.addProperty("task_id", std::string("go_to_orientation"));
    taskProperties.addProperty("target_orientation", target_orientation);
    taskProperties.addProperty("tip_name", std::string("arm_tool_link"));
    taskProperties.addProperty("damping", 0.2);
    taskProperties.addProperty("signal_reference", signal_type);

    pal_wbc_msgs::PushTask srv;
    srv.request.push_task_params.params = generateTaskDescription(taskProperties);;
    srv.request.push_task_params.respect_task_id = "go_to_position";
    srv.request.push_task_params.order.order = pal_wbc_msgs::Order::After;
    srv.request.push_task_params.blend = blend;

    if (pushTaskServ.call(srv)){
      ROS_INFO_STREAM("Succesfully pushed task:");
      ROS_INFO_STREAM(srv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }

  }

  {
    //Get current stack description
    pal_wbc_msgs::GetStackDescription statusSrv;
    if (stackDescriptionServ.call(statusSrv)){
      ROS_INFO_STREAM("Stack description:");
      ROS_INFO_STREAM(statusSrv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }

  }

  {
    ros::Duration(0.2).sleep();
    pal_wbc_msgs::GetTaskError srv;
    srv.request.id = "go_to_orientation";
    getTaskErrorServ.call(srv);
    ROS_INFO_STREAM("Task: "<<srv.request.id<<" error norm is: "<<srv.response.taskError.error_norm);
    int cont = 0;
    while(srv.response.taskError.error_norm > 1e-2){
      ROS_INFO_STREAM("Task: "<<srv.request.id<<" error norm is: "<<srv.response.taskError.error_norm);
      ros::Duration(0.1).sleep();
      getTaskErrorServ.call(srv);

      if(cont > 100){
        ROS_ERROR_STREAM("Task did not converge");
        break;
      }
      else{
        ++cont;
      }

    }
  }

  {
    //Push gaze
    ROS_INFO_STREAM("Pushing new gaze task");
    property_bag::PropertyBag taskProperties("taskType", std::string("pal_wbc/GazePointKinematicMetaTask"));
    Eigen::Vector3d positionGoal(2, 0, 1.0);

    taskProperties.addProperty("task_id", std::string("gaze"));
    taskProperties.addProperty("target_position", positionGoal);
    taskProperties.addProperty("camera_frame", std::string("xtion_optical_frame"));
    taskProperties.addProperty("damping", 0.2);
    taskProperties.addProperty("signal_reference", signal_type);

    pal_wbc_msgs::PushTask srv;
    srv.request.push_task_params.params = generateTaskDescription(taskProperties);
    srv.request.push_task_params.respect_task_id = "go_to_orientation";
    srv.request.push_task_params.order.order = pal_wbc_msgs::Order::After;
    srv.request.push_task_params.blend = blend;

    if (pushTaskServ.call(srv)){
      ROS_INFO_STREAM("Succesfully pushed task:");
      ROS_INFO_STREAM(srv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }

  }

  {
    ros::Duration(0.1).sleep();
    pal_wbc_msgs::GetTaskError srv;
    srv.request.id = "gaze";
    getTaskErrorServ.call(srv);
    ROS_INFO_STREAM("Task: "<<srv.request.id<<" error norm is: "<<srv.response.taskError.error_norm);
    int cont = 0;
    while(srv.response.taskError.error_norm > 1e-2){
      ROS_INFO_STREAM("Task: "<<srv.request.id<<" error norm is: "<<srv.response.taskError.error_norm);
      ros::Duration(0.1).sleep();
      getTaskErrorServ.call(srv);

      if(cont > 100){
        ROS_ERROR_STREAM("Task did not converge");
        break;
      }
      else{
        ++cont;
      }

    }
  }

  //Pop the gaze task right arm
  {
    ROS_INFO_STREAM("Poping orientation Task");
    pal_wbc_msgs::PopTask srv;
    srv.request.name = "gaze";
    srv.request.blend = blend;
    if (popTaskServ.call(srv)){
      ROS_INFO_STREAM(srv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }
  }

  /// @bug
  ros::Duration(5.).sleep();

  //Pop the goto orientation task right arm
  {
    ROS_INFO_STREAM("Poping orientation Task");
    pal_wbc_msgs::PopTask srv;
    srv.request.name = "go_to_orientation";
    srv.request.blend = blend;

    if (popTaskServ.call(srv)){
      ROS_INFO_STREAM(srv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }
  }

  /// @bug
  ros::Duration(5.0).sleep();

  //Pop the go to position task right arm
  {
    ROS_INFO_STREAM("Poping position Task");
    pal_wbc_msgs::PopTask srv;
    srv.request.name = "go_to_position";
    srv.request.blend = blend;

    if (popTaskServ.call(srv)){
      ROS_INFO_STREAM(srv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }
  }

  /*
  //Pushing a go to position topic reference type
  //Pop the go to position task right arm
  {
    //Push the goto position right arm task
    ROS_INFO_STREAM("Pushing new gotoPosition task");
    property_bag::PropertyBag taskProperties("taskType", std::string("pal_wbc/GoToPositionMetaTask"));

    ///////////////
    taskProperties.addProperty("task_id", std::string("go_to_position"));
    taskProperties.addProperty("signal_reference", std::string("interactive_marker"));
    taskProperties.addProperty("topic_name", std::string("go_to_position_topic"));
    taskProperties.addProperty("tip_name", std::string("arm_tool_link"));
    taskProperties.addProperty("damping", 0.2);

    pal_wbc_msgs::PushTask srv;
    srv.request.push_task_params.params = generateTaskDescription(taskProperties);;
    srv.request.push_task_params.respect_task_id = "self_collision";
    srv.request.push_task_params.order.order = pal_wbc_msgs::Order::After;
    if (pushTaskServ.call(srv)){
      ROS_INFO_STREAM("Succesfully pushed task:");
      ROS_INFO_STREAM(srv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }
  }
  */

  return 0;

}
