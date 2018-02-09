#include <Eigen/Dense>
#include <pal_wbc_controller/task_abstract.h>
#include <pal_wbc_controller/stack_of_tasks_kinematic.h>
#include <wbc_tasks/joint_pos_limit_kinematic_task.h>
#include <wbc_tasks/constraint_kinematic_task.h>
#include <wbc_tasks/reference_kinematic_task.h>
#include <wbc_tasks/self_collision_kinematic_task.h>
#include <wbc_tasks/self_collision_safety_kinematic_task.h>
#include <pal_wbc_controller/generic_meta_task.h>
#include <pluginlib/class_list_macros.h>
#include <wbc_tasks/kinematic/differntial_drive_kinematic_task.h>
#include <wbc_tasks/kinematic/differntial_drive_wheel_velocity_limit_kinematic_task.h>
#include <pal_utils/permutation.h>

using namespace pal_wbc;


// Default stack with fixed base
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

    get_default_reference_from_param_server(default_reference_joints, default_reference_posture);

    ReferenceKinematicTaskAllJointsMetaTaskPtr default_reference(
        new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(), default_reference_joints,
                                                    default_reference_posture, nh, 2.));
    stack->pushTask("default_reference", default_reference);

    return true;
  }
};

PLUGINLIB_EXPORT_CLASS(tiago_fixed_base_stack, StackConfigurationKinematic);


// Default stack with mobile base
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


    // 2. Diff drvie task
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

    get_default_reference_from_param_server(default_reference_joints, default_reference_posture);
    ReferenceKinematicTaskAllJointsMetaTaskPtr default_reference(
        new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(), default_reference_joints,
                                                    default_reference_posture, nh, 2.));
    default_reference->setDamping(0.1);
    stack->pushTask("default_reference", default_reference);

    return true;
  }  
};

PLUGINLIB_EXPORT_CLASS(tiago_mobile_base_stack, StackConfigurationKinematic);