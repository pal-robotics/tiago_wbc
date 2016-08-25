#include <Eigen/Dense>

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
//#include <wbc_tasks/environment_collision_task.h>
#include <pal_wbc_controller/generic_meta_task.h>
#include <pluginlib/class_list_macros.h>
//#include <pal_collision_tools/fcl/FCLCollisionEnvironment.h>
//#include <pal_collision_tools/fcl/FCLsyntheticCollisionObjects.h>

class tiago_environment_collision_stack: public StackConfigurationKinematic{
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

        /*
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
        */

        std::vector<std::string> collisionCheckingLinks;
        collisionCheckingLinks.push_back("arm_5_link");
        collisionCheckingLinks.push_back("arm_4_link");
        collisionCheckingLinks.push_back("arm_2_link");
        collisionCheckingLinks.push_back("arm_1_link");
        collisionCheckingLinks.push_back("base_link");
        collisionCheckingLinks.push_back("torso_lift_link");
        collisionCheckingLinks.push_back("head_2_link");

        collisionCheckingLinks.push_back("arm_3_link");
        collisionCheckingLinks.push_back("arm_6_link");
        collisionCheckingLinks.push_back("arm_7_link");
        collisionCheckingLinks.push_back("gripper_link");
        collisionCheckingLinks.push_back("gripper_left_finger_link");
        collisionCheckingLinks.push_back("gripper_right_finger_link");

        CollisionMatrixPtr ColMatrix( new CollisionMatrix());
        FCLCollisionEnvironmentPtr ce(new FCLCollisionEnvironment(RigidBodyDynamics::FloatingBaseType::FixedBase,
                                                                  stack->getSubTreeTipsRobotModel(), ColMatrix));

        /// @todo Add fake table to collision environment
        // Add synthetic table to collision environment
        std::vector<std::unique_ptr<CollisionObjectBase> > table;
        eMatrixHom tableTf = createMatrix(Eigen::Quaterniond::Identity(), eVector3(0.7, 0., 0));
        createTableFCL("table", 0.8, tableTf, table, Eigen::Vector3d(1,0,0));

        /*std::vector<std::unique_ptr<CollisionObjectBase> > tableObject;
        for(size_t i = 0; i < table.size(); ++i){
          std::unique_ptr<CollisionObjectBase> c = std::move(table[i]);
          tableObject.push_back(c);
        }*/

        //ce->addCollisionObjectoToEnvironment(tableObject);

        /*EnvironmentCollisionMetaTaskPtr envirnoment_collision_task(
              new EnvironmentCollisionMetaTask(*stack.get(), ce, collisionCheckingLinks, nh));

        envirnoment_collision_task->setDamping(0.1);
        stack->pushTask(envirnoment_collision_task);*/

        std::string sourceData; //either "topic" or "interactive_marker"
        nh.param<std::string>("source_data", sourceData, "interactive_marker");

        // 4. Position Target Reference for right and left arm
        GoToPositionMetaTaskPtr go_to_position_arm(new GoToPositionMetaTask(*stack.get(), "arm_7_link", "interactive_marker", nh));
        //GoToSplinePositionMetaTaskPtr go_to_position_arm(new GoToSplinePositionMetaTask(*stack.get(), "arm_7_link", sourceData, nh));
        go_to_position_arm->setDamping(0.1);
        stack->pushTask(TaskAbstractPtr(go_to_position_arm));

        //GoToOrientationMetaTaskPtr go_to_orientation_arm(new GoToOrientationMetaTask(*stack.get(), "arm_7_link", sourceData, nh));
        //GoToSplineOrientationMetaTaskPtr go_to_orientation_arm(new GoToSplineOrientationMetaTask(*stack.get(), "arm_7_link", sourceData, nh));
        //go_to_orientation_arm->setDamping(0.1);
        //stack->pushTask(TaskAbstractPtr(go_to_orientation_arm));


        //  4. Position Target Reference for right and left arm
        //GoToPoseMetaTaskPtr go_to_pose_arm(new GoToPoseMetaTask(*stack.get(), "arm_7_link", sourceData, nh));
        //go_to_pose_arm->setDamping(0.1);
        //stack->pushTask(TaskAbstractPtr(go_to_pose_arm));

//        //Gaze task
//        GazePointKinematicMetaTaskPtr gaze_task(new GazePointKinematicMetaTask(*stack.get(), "xtion_optical_frame", sourceData, nh));
//        gaze_task->setDamping(0.1);
//        stack->pushTask(gaze_task);

    }
};

PLUGINLIB_EXPORT_CLASS(tiago_environment_collision_stack, StackConfigurationKinematic);
