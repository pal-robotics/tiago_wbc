#include <Eigen/Dense>
#include <pal_wbc_controller/StackOfTasksKinematic.h>
#include <pal_wbc_controller/generic_meta_task.h>

#include <wbc_tasks/joint_pos_limit_kinematic_task.h>
#include <wbc_tasks/constraint_kinematic_task.h>
#include <wbc_tasks/go_to_kinematic_task.h>
#include <wbc_tasks/go_to_spline_kinematic_task.h>
#include <wbc_tasks/go_to_relative_kinematic_task.h>
#include <wbc_tasks/com_kinematic_task.h>
#include <wbc_tasks/com_stabilizer_kinematic_task.h>
#include <wbc_tasks/momentum_kinematic_task.h>
#include <wbc_tasks/reference_kinematic_task.h>
#include <wbc_tasks/gaze_kinematic_task.h>
#include <wbc_tasks/gaze_spline_kinematic_task.h>
#include <wbc_tasks/self_collision_kinematic_task.h>
#include <wbc_tasks/self_collision_safety_kinematic_task.h>
#include <pluginlib/class_list_macros.h>

using namespace pal_wbc;

class reemc_rpc_stack: public StackConfigurationKinematic{
public:

    void setupStack(StackOfTasksKinematicPtr stack, ros::NodeHandle &nh){

        std::vector<std::string> default_reference_joints;
        default_reference_joints.push_back("arm_left_1_joint");
        default_reference_joints.push_back("arm_left_2_joint");
        default_reference_joints.push_back("arm_left_3_joint");
        default_reference_joints.push_back("arm_left_4_joint");
        default_reference_joints.push_back("arm_left_5_joint");
        default_reference_joints.push_back("arm_left_6_joint");
        default_reference_joints.push_back("arm_left_7_joint");
        default_reference_joints.push_back("arm_right_1_joint");
        default_reference_joints.push_back("arm_right_2_joint");
        default_reference_joints.push_back("arm_right_3_joint");
        default_reference_joints.push_back("arm_right_4_joint");
        default_reference_joints.push_back("arm_right_5_joint");
        default_reference_joints.push_back("arm_right_6_joint");
        default_reference_joints.push_back("arm_right_7_joint");
        default_reference_joints.push_back("leg_left_4_joint");
        default_reference_joints.push_back("leg_right_4_joint");
        default_reference_joints.push_back("head_1_joint");
        default_reference_joints.push_back("head_2_joint");

        Eigen::VectorXd default_reference_posture(default_reference_joints.size());
        default_reference_posture.setZero();
        default_reference_posture(3) = 0.5;
        default_reference_posture(10) = 0.5;
        default_reference_posture(1) = 0.5;
        default_reference_posture(8) = 0.5;
        default_reference_posture(14) = 0.8;
        default_reference_posture(15) = 0.8;

        // 1. Joint and velocity limits
        JointPositionLimitKinematicAllJointsMetaTaskPtr joint_position_limit_task(
                    new JointPositionLimitKinematicAllJointsMetaTask(*stack.get(),
                                                                     stack->getJointPositionLimitMin(), stack->getJointPositionLimitMax(),
                                                                     stack->getJointVelocityLimitMin(), stack->getJointVelocityLimitMax(),
                                                                     stack->getJointNames(),
                                                                     1.0,
                                                                     false,
                                                                     nh));
        stack->pushTask(joint_position_limit_task);

        // 2. Constraint the COM and left, right foot
        GoToPoseMetaTaskPtr left_foot_constraint (new GoToPoseMetaTask(*stack.get(), "leg_left_6_link", "none", nh) );
        GoToPoseMetaTaskPtr right_foot_constraint (new GoToPoseMetaTask(*stack.get(), "leg_right_6_link", "none", nh) );

        std::vector<TaskAbstractPtr> constraint_tasks;
        constraint_tasks.push_back(left_foot_constraint);
        constraint_tasks.push_back(right_foot_constraint);
        GenericMetaTaskPtr constraint_metatask(new GenericMetaTask(constraint_tasks, stack->getStateSize()));
        stack->pushTask(TaskAbstractPtr(constraint_metatask));

        ConstraintFIXC0MMetaTaskPtr com_constraint (new ConstraintFIXC0MMetaTask(*stack.get(), nh));
        stack->pushTask(com_constraint);

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

      
        GoToOrientationMetaTaskPtr base_link_orientation(new GoToOrientationMetaTask(*stack.get(), "base_link", "interactive_marker", nh));
        base_link_orientation->setDamping(0.2);
        stack->pushTask(base_link_orientation);

        GoToOrientationMetaTaskPtr torso_link_orientation(new GoToOrientationMetaTask(*stack.get(), "torso_2_link", "interactive_marker", nh));
        torso_link_orientation->setDamping(0.2);
        stack->pushTask(torso_link_orientation);

        ReferenceKinematicTaskAllJointsMetaTaskPtr reference(new ReferenceKinematicTaskAllJointsMetaTask(*stack.get(),
                                                                                                         default_reference_joints,
                                                                                                         default_reference_posture,
                                                                                                         nh));
        torso_link_orientation->setDamping(0.2);
        stack->pushTask(reference);
    }
};

PLUGINLIB_EXPORT_CLASS(reemc_rpc_stack, StackConfigurationKinematic);

