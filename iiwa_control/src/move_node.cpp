/*
 * Jonathan Burkhard, SMS-Lab, ETH Zurich, Switzerland
 * Jonathan Burkhard, CSEM S.A., Alpnach Dorf, Switzerland
 *
 * Node to listen topic for a movement
 */

// TODO:
//         Create the group only once and reuse it... but not Idea so far.. 
//          => create a class and wrapp the callback function...
//          => https://answers.ros.org/question/11810/how-to-pass-arguments-tofrom-subscriber-callback-functions/

#include "move_node.h"

int main(int argc, char **argv)
{
    // std::cout << "Start..." << std::endl;
    // ros::init(argc, argv, "talker");
    ros::init(argc, argv, "move_node_listener_cpp");
    ros::NodeHandle nh; 

    // // Listener for the two messages
    ros::Subscriber sub_pose = nh.subscribe("/iiwa/moveToCartesianPose/", 30, chatterCallback_pose);
    ros::Subscriber sub_joint = nh.subscribe("/iiwa/moveToJoints/", 30, chatterCallback_joint);
    // ros::spin();
    ros::AsyncSpinner spinner(6);  // Use 4 threads
    spinner.start();

    ros::waitForShutdown();
    return 0;
}

// Fonctions

// #include <geometry_msgs/Pose.msg>
// void chatterCallback_pose(const std_msgs::Float64MultiArray::ConstPtr& msg)//std_msgs::String::ConstPtr& msg)
void chatterCallback_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
    /* 
    rostopic pub /iiwa/moveToCartesianPose std_msgs/Float64MultiArray "layout:
    dim:
    - label: ''
        size: 6
        stride: 0
    data_offset: 0
    data:
    - 0.25
    - 0.5
    - 0.7
    - 0.0
    - 0.0
    - 0.0"
    */
    bool choice = true;
    if(choice == false)
    {
        // result is a number
        int result = 0;
        bool result_plan = false;
        // moveit::planning_interface::MoveGroup group("manipulator");
        
        // geometry_msgs::Pose target_pose;
        // tf2::Quaternion q;
        // // Create this quaternion from roll/pitch/yaw (in radians)
        // q.setRPY(  msg->data[3],  msg->data[4],  msg->data[5] );  
        // target_pose.orientation.x = q[0];//0.5;
        // target_pose.orientation.y = q[1];//0.25;
        // target_pose.orientation.z = q[2];//0.45;
        // target_pose.orientation.w = q[3];//1.0;
        // target_pose.position.x = msg->data[0];//0.5;
        // target_pose.position.y = msg->data[1];//0.3;
        // target_pose.position.z = msg->data[2];//0.9;
        // // std::cout << "x: " << msg->data[0] << ", q0: " << q[0] << std::endl;
        // group.setPoseTarget(target_pose);

        
        // // Now, we call the planner to compute the plan
        // // and visualize it.
        // // Note that we are just planning, not asking move_group 
        // // to actually move the robot.
        // // moveit::planning_interface::MoveGroup::Plan my_plan;
        // // std::cout << "before planning" << std::endl;
        // moveit::planning_interface::MoveGroup::Plan my_plan;
        // result_plan = group.plan(my_plan);
        // // std::cout << "before execute" << std::endl;
        // result = group.move();
    }else
    {
        int result = 0;
        bool result_plan = false;
        moveit::planning_interface::MoveGroup group("manipulator");
        geometry_msgs::Pose target_pose = *msg;
        group.setPoseTarget(target_pose);

        // Now, we call the planner to compute the plan
        // and visualize it.
        // Note that we are just planning, not asking move_group 
        // to actually move the robot.
        // moveit::planning_interface::MoveGroup::Plan my_plan;
        // std::cout << "before planning" << std::endl;
        moveit::planning_interface::MoveGroup::Plan my_plan;
        result_plan = group.plan(my_plan);
        // std::cout << "before execute" << std::endl;
        result = group.move();

    }
}

void chatterCallback_joint(const std_msgs::Float64MultiArray::ConstPtr& msg)// std_msgs::String::ConstPtr& msg)
{
    /* 
    rostopic pub /iiwa/moveToJoints std_msgs/Float64MultiArray "layout:
    dim:
    - label: ''
        size: 6
        stride: 0
    data_offset: 0
    data:
    - 0.5
    - 0.5
    - 0.7
    - 0.0
    - 0.0
    - 0.0
    - 0.0"
    */
    // result is a number
    int result = 0;
    bool result_plan = false;
    moveit::planning_interface::MoveGroup group("manipulator");
    
    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group =
    group.getCurrentState()->getJointModelGroup("manipulator");


    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = msg->data[0];  // radians
    joint_group_positions[1] = msg->data[1];
    joint_group_positions[2] = msg->data[2];
    joint_group_positions[3] = msg->data[3];
    joint_group_positions[4] = msg->data[4];
    joint_group_positions[5] = msg->data[5];
    joint_group_positions[6] = msg->data[6];
    group.setJointValueTarget(joint_group_positions);

     // Now, we call the planner to compute the plan
    // and visualize it.
    // Note that we are just planning, not asking move_group 
    // to actually move the robot.
    // moveit::planning_interface::MoveGroup::Plan my_plan;
    // std::cout << "before planning" << std::endl;
    moveit::planning_interface::MoveGroup::Plan my_plan;
    result_plan = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    result = group.move();

    // // ROS_INFO("I heard: [%s]", msg->data.c_str());
}

