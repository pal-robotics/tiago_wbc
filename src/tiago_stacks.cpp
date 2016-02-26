#include <Eigen/Dense>
#include <rbdl/addons/rbdlUrdfParser.h>

#include <pal_wbc_controller/task_abstract.h>
#include <pal_wbc_controller/StackOfTasksKinematic.h>

#include <wbc_tasks/joint_pos_limit_kinematic_task.h>
#include <wbc_tasks/constraint_kinematic_task.h>
#include <wbc_tasks/go_to_kinematic_task.h>
#include <wbc_tasks/go_to_spline_kinematic_task.h>
#include <wbc_tasks/com_kinematic_task.h>
#include <wbc_tasks/momentum_kinematic_task.h>
#include <wbc_tasks/reference_kinematic_task.h>
#include <wbc_tasks/grasping_kinematic_task.h>
#include <wbc_tasks/self_collision_kinematic_task.h>
#include <wbc_tasks/self_collision_safety_kinematic_task.h>
#include <wbc_tasks/gaze_kinematic_task.h>
#include <pal_wbc_controller/generic_meta_task.h>
#include <pluginlib/class_list_macros.h>

class tiago_stack: public StackConfigurationKinematic{
    void setupStack(StackOfTasksKinematicPtr stack, ros::NodeHandle &nh){

        std::vector< double > joint_pos_min_override = stack->getJointPositionLimitMin();
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
        default_reference_posture(0) = 1.7;
        default_reference_posture(1) = -0.5;
        default_reference_posture(2) = -3.4;
        default_reference_posture(3) = 0.5;
        default_reference_posture(4) = 0.0;
        default_reference_posture(5) = 0.0;
        default_reference_posture(6) = 0.0;

        default_reference_posture(7) = 0.0;
        default_reference_posture(8) = 0.0;

        default_reference_posture(9) = 0.14;

        // 1. Joint and velocity limits
        JointPositionLimitKinematicAllJointsMetaTaskPtr joint_position_limit_task(
                    new JointPositionLimitKinematicAllJointsMetaTask(*stack.get(),
                                                                     joint_pos_min_override, stack->getJointPositionLimitMax(),
                                                                     stack->getJointVelocityLimitMin(), stack->getJointVelocityLimitMax(),
                                                                     stack->getJointNames(),
                                                                     1.0,
                                                                     false,
                                                                     nh));

        stack->pushTask(joint_position_limit_task);


        // Self collision
        SelfCollisionSafetyParameters sc_params;
        sc_params.min_distance = 0.08;
        sc_params.influence_distance = 0.08;
        sc_params.epsison = 0.02;
        sc_params.safety_distance = 0;
        sc_params.number_collisions = 10;
        SelfCollisionSafetyKinematicTaskPtr self_collision(new SelfCollisionSafetyKinematicTask() );
        self_collision->setUpTask(sc_params, *stack.get(), nh);
        self_collision->setDamping(0.1);

        stack->pushTask(self_collision);

        std::string sourceData; //either "topic" or "interactive_marker"
        nh.param<std::string>("source_data", sourceData, "interactive_marker");

        // 4. Position Target Reference for right and left arm
        GoToPositionMetaTaskPtr go_to_position_arm(new GoToPositionMetaTask(*stack.get(), "arm_7_link", sourceData, nh));
        //GoToSplinePositionMetaTaskPtr go_to_position_arm(new GoToSplinePositionMetaTask(*stack.get(), "arm_7_link", sourceData, nh));
        go_to_position_arm->setDamping(0.1);
        stack->pushTask(TaskAbstractPtr(go_to_position_arm));


        GoToOrientationMetaTaskPtr go_to_orientation_arm(new GoToOrientationMetaTask(*stack.get(), "arm_7_link", sourceData, nh));
        //GoToSplineOrientationMetaTaskPtr go_to_orientation_arm(new GoToSplineOrientationMetaTask(*stack.get(), "arm_7_link", sourceData, nh));
        go_to_orientation_arm->setDamping(0.1);
        stack->pushTask(TaskAbstractPtr(go_to_orientation_arm));

        //  4. Position Target Reference for right and left arm
        //GoToPoseMetaTaskPtr go_to_pose_arm(new GoToPoseMetaTask(*stack.get(), "arm_7_link", sourceData, nh));
        //go_to_pose_arm->setDamping(0.1);
        //stack->pushTask(TaskAbstractPtr(go_to_pose_arm));


        //Gaze task
        GazePointKinematicMetaTaskPtr gaze_task(new GazePointKinematicMetaTask(*stack.get(), "xtion_optical_frame", sourceData, nh));
        gaze_task->setDamping(0.1);
        stack->pushTask(gaze_task);

    }
};

PLUGINLIB_EXPORT_CLASS(tiago_stack, StackConfigurationKinematic);

/*
class tiago_reachability_stack: public StackConfiguration{
  public:

  void setupStack(StackOfTasksPtr stack){

    eVector3 r; r.setZero();
    eQuaternion E; E.setIdentity();
    tip_target = ReferenceAbstract::create("none", stack->controller_nh, "gripper_link", "/base_link", r, E);


    RigidBodyDynamics::Model &rbdl_model = stack->rbdl_model_;
    Eigen::VectorXd referece_posture(rbdl_model.dof_count);
    referece_posture.setZero();

    // 1. Joint and velocity limits
    JointPositionLimitKinematicAllJointsMetaTaskPtr joint_position_limit_task(
          new JointPositionLimitKinematicAllJointsMetaTask(stack.get(),
                                                           stack->joint_position_min, stack->joint_positoin_max,
                                                           stack->joint_vel_min, stack->joint_vel_max,
                                                           stack->joint_names,
                                                           1.0,
                                                           true));
    stack->pushTask(joint_position_limit_task);

    // 4. Position Target Reference for right and left arm
    GoToPositionMetaTaskPtr go_to_position_arm(new GoToPositionMetaTask(stack.get(), "arm_7_link", tip_target));
    go_to_position_arm->setDamping(0.1);
    stack->pushTask(TaskAbstractPtr(go_to_position_arm));

    GoToOrientationMetaTaskPtr go_to_orientation_arm(new GoToOrientationMetaTask(stack.get(), "arm_7_link", tip_target));
    go_to_position_arm->setDamping(0.1);
    stack->pushTask(TaskAbstractPtr(go_to_orientation_arm));
  }

  ReferenceAbstractPtr tip_target;

};
*/

//COMPONENT_REGISTER(StackConfiguration, tiago_reachability_stack, "tiago_reachability_stack")
