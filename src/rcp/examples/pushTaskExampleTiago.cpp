#include <iostream>
#include <ros/ros.h>
#include <pal_wbc_msgs/GetStackDescription.h>
#include <pal_wbc_msgs/GetTaskError.h>
#include <pal_wbc_msgs/PopTask.h>
#include <pal_wbc_msgs/PushTask.h>
#include <Eigen/Dense>
#include <property_bag/property_bag.h>
#include <property_bag/serialization/property_bag_boost_serialization.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

pal_wbc_msgs::TaskDescription generateTaskDescription(const property_bag::PropertyBag properties){

  pal_wbc_msgs::TaskDescription newTask;
  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  oa << properties;
  newTask.description = ss.str();

  return newTask;
}

int main(int argc, char** argv){

  ros::init(argc, argv, "wbc_reemc_hardware_test");
  ros::NodeHandle nh("~");

  ros::ServiceClient stackDescriptionServ = nh.serviceClient<pal_wbc_msgs::GetStackDescription>("/whole_body_kinematic_controler/get_stack_description");
  ros::ServiceClient popTaskServ = nh.serviceClient<pal_wbc_msgs::PopTask>("/whole_body_kinematic_controler/pop_task");
  ros::ServiceClient pushTaskServ = nh.serviceClient<pal_wbc_msgs::PushTask>("/whole_body_kinematic_controler/push_task");
  ros::ServiceClient getTaskErrorServ = nh.serviceClient<pal_wbc_msgs::GetTaskError>("/whole_body_kinematic_controler/get_task_error");

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

  {
    //Push the goto position right arm task
    ROS_INFO_STREAM("Pushing new gotoPosition task");
    property_bag::PropertyBag taskProperties("taskType", std::string("pal_wbc/GoToPositionMetaTask"));
    Eigen::Vector3d positionGoal(0.24374, -0.3, 1.0);
    //Eigen::Vector3d positionGoal(0.34955, 0.28446, 0.2325);

    taskProperties.addProperty("taskId", std::string("go_to_position"));
    taskProperties.addProperty("target_position", positionGoal);
    taskProperties.addProperty("tip_name", std::string("arm_tool_link"));
    taskProperties.addProperty("damping", 0.2);

    pal_wbc_msgs::TaskDescription newTask = generateTaskDescription(taskProperties);
    pal_wbc_msgs::PushTask srv;
    srv.request.push_task_params.task = newTask;
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


  {
    ros::Duration(0.1).sleep();
    pal_wbc_msgs::GetTaskError srv;
    srv.request.id = 1;
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
    //Push the goto orientation task right arm task
    ROS_INFO_STREAM("Pushing new GotoOrientation task");
    property_bag::PropertyBag taskProperties("taskType", std::string("pal_wbc/GoToOrientationMetaTask"));
    Eigen::Quaterniond target_orientation;

    target_orientation.w() = 1.0;
    target_orientation.x() = 0.0;
    target_orientation.y() =  0.0;
    target_orientation.z() = 0.0;

    taskProperties.addProperty("taskId", std::string("go_to_orientation"));
    taskProperties.addProperty("target_orientation", target_orientation);
    taskProperties.addProperty("tip_name", std::string("arm_tool_link"));
    taskProperties.addProperty("damping", 0.2);

    pal_wbc_msgs::TaskDescription newTask = generateTaskDescription(taskProperties);
    pal_wbc_msgs::PushTask srv;
    srv.request.push_task_params.task = newTask;
    srv.request.push_task_params.respect_task_id = "go_to_position";
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

  {
    ros::Duration(0.2).sleep();
    pal_wbc_msgs::GetTaskError srv;
    srv.request.id = 2;
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
   /*

  {
    //Push gaze
    ROS_INFO_STREAM("Pushing new gaze task");
    property_bag::PropertyBag taskProperties("taskType", std::string("pal_wbc/GazePointKinematicMetaTask"));
    Eigen::Vector3d positionGoal(0.24374, -0.3, 1.0);

    taskProperties.addProperty("target_position", positionGoal);
    taskProperties.addProperty("tip_name", std::string("xtion_optical_frame"));
    taskProperties.addProperty("damping", 0.2);

    pal_wbc_controller::TaskDescription newTask = generateTaskDescription(taskProperties);
    pal_wbc_controller::PushTask srv;
    srv.request.task = newTask;
    srv.request.orderInStack = 3;
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
    pal_wbc_controller::GetTaskError srv;
    srv.request.id = 3;
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
    pal_wbc_controller::PopTask srv;
    srv.request.orderInStack = 3;
    if (popTaskServ.call(srv)){
      ROS_INFO_STREAM(statusSrv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }
  }

  //Pop the goto orientation task right arm
  {
    ROS_INFO_STREAM("Poping orientation Task");
    pal_wbc_controller::PopTask srv;
    srv.request.orderInStack = 2;
    if (popTaskServ.call(srv)){
      ROS_INFO_STREAM(statusSrv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }
  }

  ros::Duration(5.0).sleep();

  //Pop the go to position task right arm
  {
    ROS_INFO_STREAM("Poping position Task");
    pal_wbc_controller::PopTask srv;
    srv.request.orderInStack = 1;
    if (popTaskServ.call(srv)){
      ROS_INFO_STREAM(statusSrv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }
  }

  //Pushing a go to position topic reference type
  //Pop the go to position task right arm
  {
    //Push the goto position right arm task
    ROS_INFO_STREAM("Pushing new gotoPosition task");
    property_bag::PropertyBag taskProperties("taskType", std::string("pal_wbc/GoToPositionMetaTask"));

    ///////////////
    taskProperties.addProperty("reference_type", std::string("topic"));
    taskProperties.addProperty("topic_name", std::string("go_to_position_topic"));
    taskProperties.addProperty("tip_name", std::string("arm_tool_link"));
    taskProperties.addProperty("damping", 0.2);

    pal_wbc_controller::TaskDescription newTask = generateTaskDescription(taskProperties);
    pal_wbc_controller::PushTask srv;
    srv.request.task = newTask;
    srv.request.orderInStack = 1;
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
