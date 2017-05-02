#include <Eigen/Dense>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include <pal_wbc_controller/task_abstract.h>
#include <pal_wbc_controller/StackOfTasksKinematic.h>

#include <wbc_tasks/joint_pos_limit_kinematic_task.h>
#include <wbc_tasks/constraint_kinematic_task.h>
#include <wbc_tasks/go_to_kinematic_task.h>
#include <wbc_tasks/go_to_spline_kinematic_task.h>
#include <wbc_tasks/com_kinematic_task.h>
#include <wbc_tasks/momentum_kinematic_task.h>
#include <wbc_tasks/reference_kinematic_task.h>
#include <wbc_tasks/self_collision_kinematic_task.h>
#include <wbc_tasks/self_collision_safety_kinematic_task.h>
#include <wbc_tasks/gaze_kinematic_task.h>
#include <pal_wbc_controller/generic_meta_task.h>
#include <pluginlib/class_list_macros.h>

using namespace pal_wbc;

class marco_rpc_stack: public StackConfigurationKinematic{
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
        default_reference_posture(0) = 0.11;//1.7;
        default_reference_posture(1) = -0.51;//-0.5;
        default_reference_posture(2) = -3.22;//-3.4;
        default_reference_posture(3) = 2.0;//0.5;
        default_reference_posture(4) = 1.91;//3.4;
        default_reference_posture(5) = 0.36;//0.0;
        default_reference_posture(6) = 1.08;//0.0;

        default_reference_posture(7) = 0.0;
        default_reference_posture(8) = 0.0;

        default_reference_posture(9) = 0.3;//0.14;

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


        ReferenceKinematicTaskAllJointsMetaTaskPtr default_reference(new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(),
                                                                                                                 default_reference_joints,
                                                                                                                 default_reference_posture,
                                                                                                                 nh));
        stack->pushTask(default_reference);


    }
};

PLUGINLIB_EXPORT_CLASS(marco_rpc_stack, StackConfigurationKinematic);
