#include <rclcpp/rclcpp.hpp>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/msg/point_stamped.h>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_tutorial");

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

    static const std::string PLANNING_GROUP = "arm";
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

    geometry_msgs::msg::PoseStamped target_pose1;
    target_pose1.header.frame_id = "world";
    target_pose1.pose.orientation.x = 0.044;
    target_pose1.pose.orientation.y = 0.993;
    target_pose1.pose.orientation.z = -0.050;
    target_pose1.pose.orientation.w = -0.093;
    target_pose1.pose.position.x = -0.150;
    target_pose1.pose.position.y = 0.048;
    target_pose1.pose.position.z = 1.125;

    geometry_msgs::msg::PoseStamped target_pose2;
    target_pose2.header.frame_id = "world";
    target_pose2.pose.orientation.x = 0.819;
    target_pose2.pose.orientation.y = -0.574;
    target_pose2.pose.orientation.z = 0.004;
    target_pose2.pose.orientation.w = 0.002;
    target_pose2.pose.position.x = 0.111;
    target_pose2.pose.position.y = -0.306;
    target_pose2.pose.position.z = 1.125;

    geometry_msgs::msg::PoseStamped target_pose3;
    target_pose3.header.frame_id = "world";
    target_pose3.pose.orientation.x = 0.509;
    target_pose3.pose.orientation.y = -0.497;
    target_pose3.pose.orientation.z = 0.489;
    target_pose3.pose.orientation.w = 0.505;
    target_pose3.pose.position.x = -0.052;
    target_pose3.pose.position.y = -0.652;
    target_pose3.pose.position.z = 1.125;

    while(rclcpp::ok())
    {
        planning_components->setGoal(target_pose1, "prbt_tool0");
        auto plan = planning_components->plan();

        if(plan)
        {
            bool blocking = true;
            moveit_cpp_ptr->execute(PLANNING_GROUP, plan.trajectory, blocking);
            // moveit_controller_manager::ExecutionStatus result = moveit_cpp_ptr->execute(plan.trajectory, blocking, CONTROLLERS);
        }

        rclcpp::sleep_for(std::chrono::seconds(1));
        planning_components->setGoal(target_pose2, "prbt_tool0");
        auto plan2 = planning_components->plan();

        if(plan2)
        {
            bool blocking = true;
            moveit_cpp_ptr->execute(PLANNING_GROUP, plan2.trajectory, blocking);
        }

        rclcpp::sleep_for(std::chrono::seconds(1));

        planning_components->setGoal(target_pose3, "prbt_tool0");
        auto plan3 = planning_components->plan();

        if(plan3)
        {
            bool blocking = true;
            moveit_cpp_ptr->execute(PLANNING_GROUP, plan3.trajectory, blocking);
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(LOGGER, "Shutting down.");
    rclcpp::shutdown();
    return 0;
}