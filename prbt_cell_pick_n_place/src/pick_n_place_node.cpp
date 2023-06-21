#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/msg/point_stamped.h>

#include "schunk_command_interface/action/egp40_command.hpp"

using GripperCommand = schunk_command_interface::action::Egp40Command;
using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_tutorial");
static const std::string PLANNING_GROUP = "arm";

void feedback_callback(GoalHandleGripperCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback)
{
    RCLCPP_INFO(LOGGER, "Received feedback: %s", feedback->operation_state.c_str());
}

void goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle)
{
    using namespace std::placeholders;
    if(!goal_handle)
    {
        RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
    }
    else
    {
        RCLCPP_INFO(LOGGER, "Goal accepted by server, waiting for result");
    }
}

void result_callback(const GoalHandleGripperCommand::WrappedResult & result)
{
    using namespace std::placeholders;
    switch(result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(LOGGER, "Goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(LOGGER, "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(LOGGER, "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(LOGGER, "Unknown result code");
            return;
    }
    RCLCPP_INFO(LOGGER, "Result received");
}

void execute_pose(moveit_cpp::MoveItCppPtr& moveit_cpp, moveit_cpp::PlanningComponentPtr &planning_component, const geometry_msgs::msg::PoseStamped& pose)
{
    planning_component->setGoal(pose, "prbt_tool0");
    auto plan_solution = planning_component->plan();
    if (plan_solution)
    {
        RCLCPP_INFO(LOGGER, "Planning succeeded");
        moveit_cpp->execute(PLANNING_GROUP, plan_solution.trajectory, true);
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Planning failed");
    }
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    RCLCPP_INFO(LOGGER, "Initialize node");

    // This enables loading undeclared parameters
    // best practice would be to declare parameters in the corresponding classes
    // and provide descriptions about expected use
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    auto gripper_client = rclcpp_action::create_client<GripperCommand>(node, "/gripper_command");

    auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(goal_response_callback, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(feedback_callback, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(result_callback, std::placeholders::_1);

    static const std::string LOGNAME = "moveit_cpp_tutorial";
    // ros2_controllers
    static const std::vector<std::string> CONTROLLERS(1, "arm_controller");

    /* Otherwise robot with zeros joint_states */
    rclcpp::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(LOGGER, "Starting MoveIt Tutorials...");

    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
    moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
    auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    auto robot_start_state = planning_components->getStartState();
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);
    
    planning_components->setStartStateToCurrentState();

    tf2::Quaternion q;
    q.setRPY(3.142, -0.009, -3.137);

    geometry_msgs::msg::PoseStamped pick_pose;
    pick_pose.header.frame_id = "world";
    pick_pose.pose.orientation.x = q.getX();
    pick_pose.pose.orientation.y = q.getY();
    pick_pose.pose.orientation.z = q.getZ();
    pick_pose.pose.orientation.w = q.getW();
    pick_pose.pose.position.x = 0.007;
    pick_pose.pose.position.y = 0.018;
    pick_pose.pose.position.z = 1.147;

    geometry_msgs::msg::PoseStamped pre_pick_pose;
    pre_pick_pose.pose.position.z -= 0.1;

    geometry_msgs::msg::PoseStamped place_pose;
    place_pose.header.frame_id = "world";
    place_pose.pose.orientation.x = q.getX();
    place_pose.pose.orientation.y = q.getY();
    place_pose.pose.orientation.z = q.getZ();
    place_pose.pose.orientation.w = q.getW();
    place_pose.pose.position.x = 0.01;
    place_pose.pose.position.y = 0.018;
    place_pose.pose.position.z = 1.147;

    geometry_msgs::msg::PoseStamped pre_place_pose;
    pre_place_pose.pose.position.z -= 0.1;

    auto gripper_msg = schunk_command_interface::action::Egp40Command::Goal();

    while(rclcpp::ok())
    {
        execute_pose(moveit_cpp_ptr, planning_components, pre_pick_pose);
        gripper_msg.is_open = true;
        auto gripper_handle = gripper_client->async_send_goal(gripper_msg, send_goal_options);
        // executor.spin_until_future_complete(gripper_handle);
        gripper_handle.wait_for(std::chrono::seconds(1));
        if ( != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(LOGGER, "Failed to set goal");
            return 1;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        execute_pose(moveit_cpp_ptr, planning_components, pick_pose);
        gripper_msg.is_open = false;
        gripper_handle = gripper_client->async_send_goal(gripper_msg, send_goal_options);
        // executor.spin_until_future_complete(gripper_handle);
        if (executor.spin_until_future_complete(gripper_handle) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(LOGGER, "Failed to set goal");
            return 1;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));

        execute_pose(moveit_cpp_ptr, planning_components, pre_pick_pose);
        rclcpp::sleep_for(std::chrono::seconds(1));

        execute_pose(moveit_cpp_ptr, planning_components, pre_place_pose);
        rclcpp::sleep_for(std::chrono::seconds(1));

        execute_pose(moveit_cpp_ptr, planning_components, place_pose);
        gripper_msg.is_open = true;
        gripper_handle = gripper_client->async_send_goal(gripper_msg, send_goal_options);
        // executor.spin_until_future_complete(gripper_handle);
        if (executor.spin_until_future_complete(gripper_handle) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(LOGGER, "Failed to set goal");
            return 1;
        }
    }

    RCLCPP_INFO(LOGGER, "Shutting down.");
    rclcpp::shutdown();
    return 0;
}