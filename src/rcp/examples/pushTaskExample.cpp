#include <iostream>
#include <ros/ros.h>
#include <pal_wbc_controller/GetStackDescription.h>
#include <pal_wbc_controller/GetTaskError.h>
#include <pal_wbc_controller/PopTask.h>
#include <pal_wbc_controller/PushTask.h>
#include <property_bag/property_bag.h>
#include <Eigen/Dense>

int main(int argc, char** argv){

  ros::init(argc, argv, "wbc_reemc_hardware_test");
  ros::NodeHandle nh("~");

  ros::ServiceClient stackDescriptionServ = nh.serviceClient<pal_wbc_controller::GetStackDescription>("/whole_body_kinematic_controler/get_stack_description");
  ros::ServiceClient popTaskServ = nh.serviceClient<pal_wbc_controller::PopTask>("/whole_body_kinematic_controler/pop_task");
  ros::ServiceClient pushTaskServ = nh.serviceClient<pal_wbc_controller::PushTask>("/whole_body_kinematic_controler/push_task");
  ros::ServiceClient getTaskErrorServ = nh.serviceClient<pal_wbc_controller::GetTaskError>("/whole_body_kinematic_controler/get_task_error");

  //Get current stack description
  pal_wbc_controller::GetStackDescription statusSrv;
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
    property_bag::PropertyBag taskProperties;
    taskProperties.addProperty("taskType", std::string("goToPosition"));
    Eigen::Vector3d positionGoal(0.24374, -0.047639, 0.20809);
    //Eigen::Vector3d positionGoal(0.34955, 0.28446, 0.2325);

    taskProperties.addProperty("target_position", positionGoal);
    taskProperties.addProperty("link_name", std::string("arm_left_7_link"));
    taskProperties.addProperty("damping", 0.2);

    pal_wbc_controller::TaskDescription newTask;
    newTask.description = taskProperties.getSerializedString();
    pal_wbc_controller::PushTask srv;
    srv.request.task = newTask;
    srv.request.orderInStack = 4;
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
    srv.request.id = 4;
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

  //Push the goto orientation task right arm task
  {
    //Push the goto position right arm task
    ROS_INFO_STREAM("Pushing new GotoOrientation task");
    property_bag::PropertyBag taskProperties("taskType", std::string("goToOrientation"));
    Eigen::Quaternion<double> target_orientation;

    target_orientation.w() = -0.57993;
    target_orientation.x() = 0.30637;
    target_orientation.y() =  0.66329;
    target_orientation.z() = -0.36037;

    taskProperties.addProperty("target_orientation", target_orientation);
    taskProperties.addProperty("link_name", std::string("arm_left_7_link"));
    taskProperties.addProperty("damping", 0.2);

    pal_wbc_controller::TaskDescription newTask;
    newTask.description = taskProperties.getSerializedString();
    pal_wbc_controller::PushTask srv;
    srv.request.task = newTask;
    srv.request.orderInStack = 5;
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
    pal_wbc_controller::GetTaskError srv;
    srv.request.id = 5;
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
    property_bag::PropertyBag taskProperties("taskType", std::string("gazeTo"));
    Eigen::Vector3d positionGoal(0.24374, -0.047639, 0.20809);

    taskProperties.addProperty("target_position", positionGoal);
    taskProperties.addProperty("link_name", std::string("stereo_optical_frame"));
    taskProperties.addProperty("damping", 0.2);

    pal_wbc_controller::TaskDescription newTask;
    newTask.description = taskProperties.getSerializedString();
    pal_wbc_controller::PushTask srv;
    srv.request.task = newTask;
    srv.request.orderInStack = 6;
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
    srv.request.id = 6;
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



  //Pop the goto orientation task right arm
  {
    ROS_INFO_STREAM("Poping orientation Task");
    pal_wbc_controller::PopTask srv;
    srv.request.orderInStack = 6;
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
    srv.request.orderInStack = 5;
    if (popTaskServ.call(srv)){
      ROS_INFO_STREAM(statusSrv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }
  }

  ros::Duration(5.0).sleep();

  //Push the go to position orientation right arm
  {
    ROS_INFO_STREAM("Poping position Task");
    pal_wbc_controller::PopTask srv;
    srv.request.orderInStack = 4;
    if (popTaskServ.call(srv)){
      ROS_INFO_STREAM(statusSrv.response);
    }
    else{
      ROS_ERROR("Failed to call service ");
      return 1;
    }
  }

  return 0;

}
