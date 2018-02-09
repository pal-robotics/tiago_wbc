#include <Eigen/Dense>

#include <pal_wbc_controller/task_abstract.h>
#include <pal_wbc_controller/stack_of_tasks_kinematic.h>

#include <wbc_tasks/joint_pos_limit_kinematic_task.h>
#include <wbc_tasks/constraint_kinematic_task.h>
#include <wbc_tasks/go_to_kinematic_task.h>
#include <wbc_tasks/com_kinematic_task.h>
#include <wbc_tasks/reference_kinematic_task.h>
#include <wbc_tasks/go_to_admitance_kinematic_task.h>
#include <wbc_tasks/self_collision_kinematic_task.h>
#include <wbc_tasks/self_collision_safety_kinematic_task.h>
#include <wbc_tasks/gaze_kinematic_task.h>
#include <pal_wbc_controller/generic_meta_task.h>
#include <pluginlib/class_list_macros.h>
#include <wbc_tasks/kinematic/differntial_drive_kinematic_task.h>
#include <wbc_tasks/kinematic/differntial_drive_wheel_velocity_limit_kinematic_task.h>
#include <pal_utils/permutation.h>
#include <pal_ros_utils/reference/vector/vector_topic_reference.h>
#include <wbc_tasks/kinematic/go_to_point_ray_angle_constraint.h>
#include <wbc_tasks/kinematic/go_to_virtual_admitance_kinematic_task.h>
#include <wbc_tasks/kinematic/go_to_local_virtual_admitance_kinematic_task.h>

using namespace pal_wbc;

bool get_default_reference_from_param_server(const std::vector<std::string> &default_reference_joints,
											 Eigen::VectorXd &default_reference_posture)
{
	ros::NodeHandle nh;

	if(nh.hasParam("/whole_body_kinematic_controller/reference"))
	{
		ROS_INFO("Getting reference from param server");
		double pos = 0.0;
		nh.getParam("/whole_body_kinematic_controller/reference/arm_1_joint", pos);
		default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_1_joint"))) = pos;
	    nh.getParam("/whole_body_kinematic_controller/reference/arm_2_joint", pos);
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_2_joint"))) = pos;
	    nh.getParam("/whole_body_kinematic_controller/reference/arm_3_joint", pos);
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_3_joint"))) = pos;
	    nh.getParam("/whole_body_kinematic_controller/reference/arm_4_joint", pos);
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_4_joint"))) = pos;
	    nh.getParam("/whole_body_kinematic_controller/reference/arm_5_joint", pos);
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_5_joint"))) = pos;
	    nh.getParam("/whole_body_kinematic_controller/reference/arm_6_joint", pos);
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_6_joint"))) = pos;
	    nh.getParam("/whole_body_kinematic_controller/reference/arm_7_joint", pos);
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_7_joint"))) = pos;

	    nh.getParam("/whole_body_kinematic_controller/reference/head_1_joint", pos);
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("head_1_joint"))) = pos;
	    nh.getParam("/whole_body_kinematic_controller/reference/head_2_joint", pos);
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("head_2_joint"))) = pos;

	    nh.getParam("/whole_body_kinematic_controller/reference/torso_lift_joint", pos);
	    default_reference_posture(indexVectorThrow(default_reference_joints,
	                                               std::string("torso_lift_joint"))) = pos;
	    return true;
	}
	return false;
}

// Default stack for fixed base
class tiago_fixed_base_stack : public StackConfigurationKinematic
{
  bool setupStack(StackOfTasksKinematicPtr stack, ros::NodeHandle &nh)
  {
    std::vector<double> joint_pos_min_override = stack->getJointPositionLimitMin();
    joint_pos_min_override[stack->getJointIndex("arm_4_joint")] = 0.2;

    std::vector<std::string> default_reference_joints;
    default_reference_joints.push_back("arm_1_joint");
    default_reference_joints.push_back("arm_2_joint");
    default_reference_joints.push_back("arm_3_joint");
    default_reference_joints.push_back("arm_4_joint");
    default_reference_joints.push_back("arm_5_joint");
    default_reference_joints.push_back("arm_6_joint");
    default_reference_joints.push_back("arm_7_joint");
    default_reference_joints.push_back("head_1_joint");
    default_reference_joints.push_back("head_2_joint");
    default_reference_joints.push_back("torso_lift_joint");

    // 1. Joint and velocity limits
    JointPositionLimitKinematicAllJointsMetaTaskPtr joint_position_limit_task(
        new JointPositionLimitKinematicAllJointsMetaTask(
            *stack.get(), joint_pos_min_override, stack->getJointPositionLimitMax(),
            stack->getJointVelocityLimitMin(), stack->getJointVelocityLimitMax(),
            stack->getJointNames(), 1.0, false, nh));

    stack->pushTask("joint_limits", joint_position_limit_task);

    // 2. Self collision
    SelfCollisionSafetyParameters sc_params;
    sc_params.min_distance = 0.08;
    sc_params.influence_distance = 0.08;
    sc_params.epsison = 0.02;
    sc_params.safety_distance = 0;
    sc_params.number_collisions = 10;
    SelfCollisionSafetyKinematicTaskPtr self_collision(new SelfCollisionSafetyKinematicTask);
    self_collision->setUpTask(sc_params, *stack.get(), nh);
    self_collision->setDamping(0.1);

    stack->pushTask("self_collision", self_collision);


    // 3. Default reference
    Eigen::VectorXd default_reference_posture(default_reference_joints.size());
    default_reference_posture.setZero();

    if(!get_default_reference_from_param_server(default_reference_joints, default_reference_posture))
    {
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_1_joint"))) = 1.7;
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_2_joint"))) = -0.5;
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_3_joint"))) = -3.4;
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_4_joint"))) = 0.5;
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_5_joint"))) = 0.0;
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_6_joint"))) = 0.0;
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_7_joint"))) = 0.0;

	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("head_1_joint"))) = 0.0;
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("head_2_joint"))) = 0.0;

	    default_reference_posture(indexVectorThrow(default_reference_joints,
	                                               std::string("torso_lift_joint"))) = 0.14;
	}

    ReferenceKinematicTaskAllJointsMetaTaskPtr default_reference(
        new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(), default_reference_joints,
                                                    default_reference_posture, nh, 2.));
    stack->pushTask("default_reference", default_reference);

    return true;
  }
};

PLUGINLIB_EXPORT_CLASS(tiago_fixed_base_stack, StackConfigurationKinematic);


// Default stack for mobile base
class tiago_mobile_base_stack : public StackConfigurationKinematic
{
  bool setupStack(StackOfTasksKinematicPtr stack, ros::NodeHandle &nh)
  {
    std::vector<double> joint_pos_min_override = stack->getJointPositionLimitMin();
    joint_pos_min_override[stack->getJointIndex("arm_4_joint")] = 0.2;

    std::vector<std::string> default_reference_joints;
    default_reference_joints.push_back("arm_1_joint");
    default_reference_joints.push_back("arm_2_joint");
    default_reference_joints.push_back("arm_3_joint");
    default_reference_joints.push_back("arm_4_joint");
    default_reference_joints.push_back("arm_5_joint");
    default_reference_joints.push_back("arm_6_joint");
    default_reference_joints.push_back("arm_7_joint");
    default_reference_joints.push_back("head_1_joint");
    default_reference_joints.push_back("head_2_joint");
    default_reference_joints.push_back("torso_lift_joint");

    // 1. Joint and velocity limits
    JointPositionLimitKinematicAllJointsMetaTaskPtr joint_position_limit_task(
        new JointPositionLimitKinematicAllJointsMetaTask(
            *stack.get(), joint_pos_min_override, stack->getJointPositionLimitMax(),
            stack->getJointVelocityLimitMin(), stack->getJointVelocityLimitMax(),
            stack->getJointNames(), 1.0, false, nh));

    stack->pushTask("joint_limit", joint_position_limit_task);


    // 2. Diff drive constraints
    task_container_vector diff_drive_tasks;
    DifferentialDriveKinematicMetaTaskPtr diff_drive_task(
        new DifferentialDriveKinematicMetaTask(*stack.get(), nh));
    diff_drive_task->setDamping(0.02);
    diff_drive_tasks.push_back({ "diff_drive", diff_drive_task });

    double baseLenght;
    if (!nh.getParam("wheel_separation", baseLenght))
    {
      return false;
    }
    double wheelRadius;
    if (!nh.getParam("wheel_radius", wheelRadius))
    {
      return false;
    }
    double maxWheelSpeed = 0.1;
    DifferentialDriveWheelVelocityLimitKinematicMetaTaskPtr diff_drive_vel_limit_task(
        new DifferentialDriveWheelVelocityLimitKinematicMetaTask(
            *stack.get(), nh, baseLenght, wheelRadius, maxWheelSpeed));
    diff_drive_vel_limit_task->setWeight(0.02);
    diff_drive_tasks.push_back({ "wheel_vel_limit", diff_drive_vel_limit_task });
    GenericMetaTaskPtr diff_drive_metatask(
        new GenericMetaTask(nh, stack.get(), diff_drive_tasks, stack->getStateSize()));
    stack->pushTask("diff_drive_contraints", diff_drive_metatask);
    diff_drive_metatask->setWeight(0.02);


    // 3. Self collision
    SelfCollisionSafetyParameters sc_params;
    sc_params.min_distance = 0.08;
    sc_params.influence_distance = 0.08;
    sc_params.epsison = 0.02;
    sc_params.safety_distance = 0;
    sc_params.number_collisions = 10;
    SelfCollisionSafetyKinematicTaskPtr self_collision(new SelfCollisionSafetyKinematicTask);
    self_collision->setUpTask(sc_params, *stack.get(), nh);
    self_collision->setDamping(0.1);

    stack->pushTask("self_collision", self_collision);


    // 4. Default reference
    Eigen::VectorXd default_reference_posture(default_reference_joints.size());
    default_reference_posture.setZero();

    if(!get_default_reference_from_param_server(default_reference_joints, default_reference_posture))
    {
		default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_1_joint"))) = 0.11;
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_2_joint"))) = -0.51;
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_3_joint"))) = -3.22;
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_4_joint"))) = 2.0;
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_5_joint"))) = 1.91;
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_6_joint"))) = 0.36;
	    default_reference_posture(
	        indexVectorThrow(default_reference_joints, std::string("arm_7_joint"))) = 0.3;

	    default_reference_posture(indexVectorThrow(default_reference_joints,
	                                               std::string("torso_lift_joint"))) = 0.14;
	}

    ReferenceKinematicTaskAllJointsMetaTaskPtr default_reference(
        new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(), default_reference_joints,
                                                    default_reference_posture, nh, 2.));
    default_reference->setDamping(0.1);
    stack->pushTask("rest_joint_configuration", default_reference);

    return true;
  }
};

PLUGINLIB_EXPORT_CLASS(tiago_mobile_base_stack, StackConfigurationKinematic);


/* ********************************************************************* /
*
*
*								OLD STACKS
*
*
*
/  ******************************************************************** */



/*

class tiago_admitance_stack : public StackConfigurationKinematic
{
  bool setupStack(StackOfTasksKinematicPtr stack, ros::NodeHandle &nh)
  {
    std::vector<double> joint_pos_min_override = stack->getJointPositionLimitMin();
    joint_pos_min_override[stack->getJointIndex("arm_4_joint")] = 0.2;

    std::vector<std::string> default_reference_joints;
    default_reference_joints.push_back("arm_1_joint");
    default_reference_joints.push_back("arm_2_joint");
    default_reference_joints.push_back("arm_3_joint");
    default_reference_joints.push_back("arm_4_joint");
    default_reference_joints.push_back("arm_5_joint");
    default_reference_joints.push_back("arm_6_joint");
    default_reference_joints.push_back("arm_7_joint");
    default_reference_joints.push_back("head_1_joint");
    default_reference_joints.push_back("head_2_joint");
    default_reference_joints.push_back("torso_lift_joint");

    Eigen::VectorXd default_reference_posture(default_reference_joints.size());
    default_reference_posture.setZero();
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("arm_1_joint"))) = 0.11;
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("arm_2_joint"))) = -0.51;
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("arm_3_joint"))) = -3.22;
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("arm_4_joint"))) = 2.0;
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("arm_5_joint"))) = 1.91;
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("arm_6_joint"))) = 0.36;
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("arm_7_joint"))) = 0.3;

    default_reference_posture(indexVectorThrow(default_reference_joints,
                                               std::string("torso_lift_joint"))) = 0.14;

    // 1. Joint and velocity limits
    JointPositionLimitKinematicAllJointsMetaTaskPtr joint_position_limit_task(
        new JointPositionLimitKinematicAllJointsMetaTask(
            *stack.get(), joint_pos_min_override, stack->getJointPositionLimitMax(),
            stack->getJointVelocityLimitMin(), stack->getJointVelocityLimitMax(),
            stack->getJointNames(), 1.0, false, nh));
    stack->pushTask("joint_limits", joint_position_limit_task);
    // Self collision
    SelfCollisionSafetyParameters sc_params;
    sc_params.min_distance = 0.08;
    sc_params.influence_distance = 0.08;
    sc_params.epsison = 0.02;
    sc_params.safety_distance = 0;
    sc_params.number_collisions = 10;
    SelfCollisionSafetyKinematicTaskPtr self_collision(new SelfCollisionSafetyKinematicTask());
    self_collision->setUpTask(sc_params, *stack.get(), nh);
    self_collision->setDamping(0.1);
    stack->pushTask("self_collision", self_collision);
    std::string sourceData;  // either "topic" or "interactive_marker"
    nh.param<std::string>("source_data", sourceData, "interactive_marker");
    // 4. Position Target Reference for right and left arm

    pal_base_ros_controller::FTSensorDefinitionPtr wrist_ft;
    if (!getFT("wrist_ft_link", wrist_ft))
    {
      return false;
    }

    GoToAdmitancePositionMetaTaskPtr go_to_position_arm(new GoToAdmitancePositionMetaTask(
        *stack.get(), "wrist_ft_link", wrist_ft, sourceData, nh));
    go_to_position_arm->setDamping(0.1);
    stack->pushTask("go_to_position", go_to_position_arm);
    GoToAdmitanceOrientationMetaTaskPtr go_to_orientation_arm(new GoToAdmitanceOrientationMetaTask(
        *stack.get(), "wrist_ft_link", wrist_ft, sourceData, nh));
    go_to_orientation_arm->setDamping(0.1);
    stack->pushTask("go_to_orientation", go_to_orientation_arm);

    //    //Gaze task
    //    GazePointKinematicMetaTaskPtr gaze_task(new
    //    GazePointKinematicMetaTask(*stack.get(), "xtion_optical_frame", sourceData,
    //    nh));
    //    gaze_task->setDamping(0.1);
    //    stack->pushTask("gaze", gaze_task);

    ReferenceKinematicTaskAllJointsMetaTaskPtr default_reference(
        new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(), default_reference_joints,
                                                    default_reference_posture, nh, 2.));
    stack->pushTask("rest_joint_configuration", default_reference);

    return true;
  }
};

PLUGINLIB_EXPORT_CLASS(tiago_admitance_stack, StackConfigurationKinematic);

class tiago_virtual_admitance_stack : public StackConfigurationKinematic
{
  bool setupStack(StackOfTasksKinematicPtr stack, ros::NodeHandle &nh)
  {
    std::vector<double> joint_pos_min_override = stack->getJointPositionLimitMin();
    joint_pos_min_override[stack->getJointIndex("arm_4_joint")] = 0.2;

    std::vector<std::string> default_reference_joints;
    default_reference_joints.push_back("arm_1_joint");
    default_reference_joints.push_back("arm_2_joint");
    default_reference_joints.push_back("arm_3_joint");
    default_reference_joints.push_back("arm_4_joint");
    default_reference_joints.push_back("arm_5_joint");
    default_reference_joints.push_back("arm_6_joint");
    default_reference_joints.push_back("arm_7_joint");
    default_reference_joints.push_back("head_1_joint");
    default_reference_joints.push_back("head_2_joint");
    default_reference_joints.push_back("torso_lift_joint");

    Eigen::VectorXd default_reference_posture(default_reference_joints.size());
    default_reference_posture.setZero();
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("arm_1_joint"))) = 0.11;
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("arm_2_joint"))) = -0.51;
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("arm_3_joint"))) = -3.22;
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("arm_4_joint"))) = 2.0;
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("arm_5_joint"))) = 1.91;
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("arm_6_joint"))) = 0.36;
    default_reference_posture(
        indexVectorThrow(default_reference_joints, std::string("arm_7_joint"))) = 0.3;

    default_reference_posture(indexVectorThrow(default_reference_joints,
                                               std::string("torso_lift_joint"))) = 0.14;

    // 1. Joint and velocity limits
    JointPositionLimitKinematicAllJointsMetaTaskPtr joint_position_limit_task(
        new JointPositionLimitKinematicAllJointsMetaTask(
            *stack.get(), joint_pos_min_override, stack->getJointPositionLimitMax(),
            stack->getJointVelocityLimitMin(), stack->getJointVelocityLimitMax(),
            stack->getJointNames(), 1.0, false, nh));
    stack->pushTask("joint_limits", joint_position_limit_task);
    // Self collision
    SelfCollisionSafetyParameters sc_params;
    sc_params.min_distance = 0.08;
    sc_params.influence_distance = 0.08;
    sc_params.epsison = 0.02;
    sc_params.safety_distance = 0;
    sc_params.number_collisions = 10;
    SelfCollisionSafetyKinematicTaskPtr self_collision(new SelfCollisionSafetyKinematicTask());
    self_collision->setUpTask(sc_params, *stack.get(), nh);
    self_collision->setDamping(0.1);
    stack->pushTask("self_collision", self_collision);
    std::string sourceData;  // either "topic" or "interactive_marker"
    nh.param<std::string>("source_data", sourceData, "interactive_marker");
    // 4. Position Target Reference for right and left arm

    pal_base_ros_controller::FTSensorDefinitionPtr wrist_ft;
    if (!getFT("wrist_ft_link", wrist_ft))
    {
      return false;
    }

    task_container_vector pose_tasks;

    {
      task_container_vector position_tasks;

      double linear_mass = 1.;
      double linear_spring = 80.;
      double force_filter_gain = 0.1;
      double linear_damping = 60.;

    //  
    //  //          GoToVirtualAdmitancePositionMetaTaskPtr go_to_position_arm(
    //  //                new GoToVirtualAdmitancePositionMetaTask(*stack.get(),
    //  "wrist_ft_link",
    //  //                                                  fts_[0], sourceData, nh,
    //  //                                                  linear_mass, linear_spring,
    //  force_filter_gain, false, linear_damping));

    //  //      GoToPositionMetaTaskPtr go_to_position_arm(
    //  //            new GoToPositionMetaTask(*stack.get(), "wrist_ft_link",
    //  //                                     sourceData, nh));
    //  {
    //    GoToKinematicTaskPtr go_to_position_arm(new GoToKinematicTask());
    //    GoToKinematicParameters kinematic_params;
    //    kinematic_params.tip_name = "wrist_ft_link";
    //    kinematic_params.signal_reference_type = sourceData;

    //  kinematic_params.coordinates = {TaskAbstract::X, TaskAbstract::Y,
    //  TaskAbstract::Z};
    //    for(unsigned int i=0; i<kinematic_params.coordinates.size(); ++i){
    //      kinematic_params.lower_bound.push_back(0.0);
    //      kinematic_params.upper_bound.push_back(0.0);
    //      kinematic_params.bound_type.push_back(Bound::BOUND_TWIN);
    //    }

    //   go_to_position_arm->setUpTask(kinematic_params, *stack.get(), nh);
    //    go_to_position_arm->setDamping(0.1);
    //    position_tasks.push_back({"go_to_position", go_to_position_arm});
    //  }
      

      {
        GoToLocalVirtualAdmitanceKinematicTaskPtr go_to_admitance_position_arm(
            new GoToLocalVirtualAdmitanceKinematicTask());

        GoToLocalVirtualAdmitanceKinematicParameters kinematic_params;

        kinematic_params.mass_ = linear_mass;
        kinematic_params.spring_ = linear_spring;
        kinematic_params.filter_gain_ = force_filter_gain;
        kinematic_params.critically_damped_ = false;
        kinematic_params.damping = linear_damping;

        kinematic_params.tip_name = "wrist_ft_link";
        kinematic_params.ft = wrist_ft;
        kinematic_params.signal_reference_type = sourceData;

        kinematic_params.coordinates = { TaskAbstract::X, TaskAbstract::Y, TaskAbstract::Z };
        for (unsigned int i = 0; i < kinematic_params.coordinates.size(); ++i)
        {
          kinematic_params.lower_bound.push_back(0.0);
          kinematic_params.upper_bound.push_back(0.0);
          kinematic_params.bound_type.push_back(Bound::BOUND_TWIN);
        }

        go_to_admitance_position_arm->setUpTask(kinematic_params, *stack.get(), nh);
        go_to_admitance_position_arm->setDamping(0.1);
        position_tasks.push_back({ "go_to_admitance_position", go_to_admitance_position_arm });
      }

      GenericMetaTaskPtr generic_position_metatask(
          new GenericMetaTask(nh, stack.get(), position_tasks, stack->getStateSize()));

      pose_tasks.push_back({ "go_to_position_metatask", generic_position_metatask });
    }

    {
      double torsional_mass = 0.05;
      double torsional_spring = 2.;
      double torque_filter_gain = 0.1;
      double torsional_damping = 2.;

      GoToVirtualAdmitanceOrientationMetaTaskPtr go_to_orientation_arm(
          new GoToVirtualAdmitanceOrientationMetaTask(
              *stack.get(), "wrist_ft_link", wrist_ft, sourceData, nh, torsional_mass,
              torsional_spring, torque_filter_gain, false, torsional_damping));


      //      go_to_orientation_arm->setWeight(1e-2);
      //          GoToOrientationMetaTaskPtr go_to_orientation_arm(
      //                new GoToOrientationMetaTask(*stack.get(), "wrist_ft_link",
      //                sourceData, nh));
      //          go_to_orientation_arm->setDamping(0.1);
      pose_tasks.push_back({ "go_to_orientation", go_to_orientation_arm });
    }

    GenericMetaTaskPtr generic_pose_metatask(
        new GenericMetaTask(nh, stack.get(), pose_tasks, stack->getStateSize()));
    stack->pushTask("pose_tasks", generic_pose_metatask);

    //    //Gaze task
    //    GazePointKinematicMetaTaskPtr gaze_task(new
    //    GazePointKinematicMetaTask(*stack.get(), "xtion_optical_frame", sourceData,
    //    nh));
    //    gaze_task->setDamping(0.1);
    //    stack->pushTask("gaze", gaze_task);

    ReferenceKinematicTaskAllJointsMetaTaskPtr default_reference(
        new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(), default_reference_joints,
                                                    default_reference_posture, nh, 2.));
    stack->pushTask("rest_joint_configuration", default_reference);

    return true;
  }
};

PLUGINLIB_EXPORT_CLASS(tiago_virtual_admitance_stack, StackConfigurationKinematic);

class tiago_dynamic_ref_torso_head_stack : public StackConfigurationKinematic
{
  bool setupStack(StackOfTasksKinematicPtr stack, ros::NodeHandle &nh)
  {
    std::vector<std::string> jointNames;
    jointNames.push_back("arm_1_joint");
    jointNames.push_back("arm_2_joint");
    jointNames.push_back("arm_3_joint");
    jointNames.push_back("arm_4_joint");
    jointNames.push_back("arm_5_joint");
    jointNames.push_back("arm_6_joint");
    jointNames.push_back("arm_7_joint");
    jointNames.push_back("head_1_joint");
    jointNames.push_back("head_2_joint");
    jointNames.push_back("torso_lift_joint");


    JointPositionLimitKinematicAllJointsMetaTaskPtr joint_limit_task(
        new JointPositionLimitKinematicAllJointsMetaTask(
            *stack.get(), stack->getJointPositionLimitMin(),
            stack->getJointPositionLimitMax(), stack->getJointVelocityLimitMin(),
            stack->getJointVelocityLimitMax(), stack->getJointNames(), 1.0, false, nh));

    stack->pushTask("joint_limits", joint_limit_task);

    SelfCollisionSafetyParameters self_col_params;
    self_col_params.min_distance = 0.08;
    self_col_params.influence_distance = 0.08;
    self_col_params.epsison = 0.02;
    self_col_params.safety_distance = 0;
    self_col_params.number_collisions = 10;
    SelfCollisionSafetyKinematicTaskPtr self_collision(new SelfCollisionSafetyKinematicTask());
    self_collision->setUpTask(self_col_params, *stack.get(), nh);
    self_collision->setDamping(0.1);

    stack->pushTask("self_collision", self_collision);

    std::vector<std::string> dynamic_torso_joint_names;
    dynamic_torso_joint_names.push_back("torso_lift_joint");

    Eigen::VectorXd default_pos_torso(dynamic_torso_joint_names.size());
    default_pos_torso[0] = 0.3;

    //    Eigen::VectorXd default_vel_torso(dynamic_torso.size());
    //    default_vel_torso[0] = 0;

    pal_robot_tools::VectorTopicReferencePtr torso_topic_dyn(new pal_robot_tools::VectorTopicReference(
        nh, "/lift_controller", dynamic_torso_joint_names, default_pos_torso));

    ReferenceKinematicTaskAllJointsMetaTaskPtr torso_control(new ReferenceKinematicTaskAllJointsMetaTask(
        *stack.get(), dynamic_torso_joint_names, torso_topic_dyn, 1.5, nh));

    //    ReferenceKinematicTaskAllJointsMetaTaskDynPtr torso_control(
    //          new ReferenceKinematicTaskAllJointsMetaTaskDyn(*stack.get(),
    //          torso_topic_dyn, dynamic_torso,
    //                                                         default_pos_torso,
    //                                                         default_vel_torso, nh));

    stack->pushTask("torso_control", torso_control);

    std::vector<std::string> dynamic_head_joint_names;
    dynamic_head_joint_names.push_back("head_1_joint");
    dynamic_head_joint_names.push_back("head_2_joint");

    Eigen::VectorXd default_pos_head(dynamic_head_joint_names.size());
    default_pos_head[0] = 0;
    default_pos_head[1] = 0;

    pal_robot_tools::VectorTopicReferencePtr head_topic_dyn(new pal_robot_tools::VectorTopicReference(
        nh, "/head_controller", dynamic_head_joint_names, default_pos_head));

    ReferenceKinematicTaskAllJointsMetaTaskPtr head_control(new ReferenceKinematicTaskAllJointsMetaTask(
        *stack.get(), dynamic_head_joint_names, head_topic_dyn, 1.5, nh));

    //    ReferenceKinematicTaskAllJointsMetaTaskDynPtr head_control(
    //          new ReferenceKinematicTaskAllJointsMetaTaskDyn(*stack.get(),
    //          head_topic_dyn, dynamic_head,
    //                                                         default_pos_head,
    //                                                         default_vel_head, nh));

    stack->pushTask("head_control", head_control);

    std::string sourceData;
    nh.param<std::string>("source_data", sourceData, "interactive_marker");

    GoToPositionMetaTaskPtr go_to_position_arm(
        new GoToPositionMetaTask(*stack.get(), "arm_tool_link", sourceData, nh));
    go_to_position_arm->setDamping(0.1);
    stack->pushTask("go_to_position", go_to_position_arm);

    GoToOrientationMetaTaskPtr go_to_orientation_arm(
        new GoToOrientationMetaTask(*stack.get(), "arm_tool_link", sourceData, nh));
    go_to_orientation_arm->setDamping(0.1);
    stack->pushTask("go_to_orientation", go_to_orientation_arm);

    std::vector<std::string> default_ref_joints;
    default_ref_joints.push_back("arm_1_joint");
    default_ref_joints.push_back("arm_2_joint");
    default_ref_joints.push_back("arm_3_joint");
    default_ref_joints.push_back("arm_4_joint");
    default_ref_joints.push_back("arm_5_joint");
    default_ref_joints.push_back("arm_6_joint");
    default_ref_joints.push_back("arm_7_joint");

    Eigen::VectorXd default_ref_pos(default_ref_joints.size());
    default_ref_pos.setZero();
    default_ref_pos[0] = 1.0;
    default_ref_pos[1] = 0.7;
    default_ref_pos[2] = -0.5;
    default_ref_pos[3] = 1.72;
    default_ref_pos[4] = 0.93;
    default_ref_pos[5] = -0.69;
    default_ref_pos[6] = 0.2;

    pal_robot_tools::VectorTopicReferencePtr abs_vec_ref(new pal_robot_tools::VectorTopicReference(
        nh, "/dynamic_ref", default_ref_joints, default_ref_pos));

    ReferenceKinematicTaskAllJointsMetaTaskPtr dyn_pos(new ReferenceKinematicTaskAllJointsMetaTask(
        *stack.get(), default_ref_joints, abs_vec_ref, 1.5, nh));

    stack->pushTask("joint_rest_reference", dyn_pos);

    return true;
  }
};

PLUGINLIB_EXPORT_CLASS(tiago_dynamic_ref_torso_head_stack, StackConfigurationKinematic);

*/