#include <iostream>
#include <ros/ros.h>
#include <wbc_rpc/GetStackDescription.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "wbc_reemc_hardware_test");
    ros::NodeHandle nh("~");

    ros::ServiceClient client = nh.serviceClient<wbc_rpc::GetStackDescription>("/whole_body_kinematic_controler/get_stack_description");
    wbc_rpc::GetStackDescription srv;
    if (client.call(srv)){
        ROS_INFO_STREAM("Stack description:");
        ROS_INFO_STREAM(srv.response);

    }
    else
    {
        ROS_ERROR("Failed to call service ");
        return 1;
    }

    return 0;

}
