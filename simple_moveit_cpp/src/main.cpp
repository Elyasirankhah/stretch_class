#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

#define ARM "stretch_arm"
int main(int argc, char** argv) {
    // init ros node
    ros::init(argc, argv, "moveit_pub");
    // Create node handler
    ros::NodeHandle n;
    // Async spinner is necessary to get current joint states.
    ros::AsyncSpinner s(1);
    s.start();

    // Create a planning group. String passed as argument must be valid group. Stretch has: stretch_arm stretch_gripper stretch_dex_wrist stretch_head
    moveit::planning_interface::MoveGroupInterface move_group_arm(ARM);

    // For joint planning like this example does this tolerence is for each joint. If planning to a target point this will be tolerence from the point, but this is not possible with this config
    move_group_arm.setGoalTolerance(0.01);
    // Scales how fast the robot joint group will move. This is from (0, 1]
    move_group_arm.setMaxVelocityScalingFactor(1.0);
    // Sets total time the robot will try to solve the inverse kinematics in seconds
    move_group_arm.setPlanningTime(5);

    // Initialize a new plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group_arm.getCurrentState();
    // Copy joint model into a local copy
    const moveit::core::JointModelGroup* joint_model_group = move_group_arm.getCurrentState()->getJointModelGroup(ARM);
    // Copy all joint positions into a vector for modifying
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Modify joint positions. The index of these can be seen by rostopic echo -n 1 /stretch_arm_controller/state
    // Then look at the joint names array
    /**
    joint_names:
    - joint_lift        <-- index 0
    - joint_arm_l3
    - joint_arm_l2
    - joint_arm_l1
    - joint_arm_l0
    - joint_wrist_yaw   <-- index 5
    */
    joint_group_positions.at(2) = 0.5;  // Lift position
    double dist = 0.5 / 6;              // Arm Extension / 4
    for (int i = 1; i < 5; i++) {
        joint_group_positions.at(i) = dist;
    }
    joint_group_positions.back() = 0;  // Gripper Yaw

    // Set target values from the vector modified before
    move_group_arm.setJointValueTarget(joint_group_positions);
    // Plan the movement from current poition to ideal position
    move_group_arm.plan(my_plan);  // This can fail
    // Move robot using plan. This is blocking
    move_group_arm.execute(my_plan);  // Blocking
    // move_group_arm.asyncExecute(my_plan); // Non blocking

    // Stops Async Spinnger
    s.stop();

    return 0;
}
