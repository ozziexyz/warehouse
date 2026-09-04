#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <warehouse_interfaces/action/follow_path.hpp>
#include <warehouse_interfaces/srv/set_robot_state.hpp>
#include <warehouse_interfaces/srv/generate_path.hpp>

using namespace warehouse_interfaces::action;
using namespace warehouse_interfaces::srv;
using namespace warehouse_interfaces::msg;
using namespace std::chrono_literals;

class NavigationManager : public rclcpp::Node {
    public:
        NavigationManager() : Node("navigation_manager") {
            RCLCPP_INFO(get_logger(), "navigation_manager node started");

            goal_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal", 
                10,
                std::bind(&NavigationManager::goal_sub_callback, this, std::placeholders::_1)
            );
            odom_sub = create_subscription<nav_msgs::msg::Odometry>(
                "/diff_drive_controller/odom", 
                10,
                std::bind(&NavigationManager::odom_callback, this, std::placeholders::_1)
            );
            pose_sub = create_subscription<geometry_msgs::msg::PoseStamped> (
                "/ground_truth/pose",
                10,
                std::bind(&NavigationManager::pose_callback, this, std::placeholders::_1)
            );
            path_pub = create_publisher<nav_msgs::msg::Path>("/path", 10);
            robot_state_client = create_client<SetRobotState>("/robot_state/set", 10);
            path_planner_client = create_client<GeneratePath>("/generate_path", 10);
            controller_client = rclcpp_action::create_client<FollowPath>(this, "/follow_path");
            timer = create_timer(0.5s, std::bind(&NavigationManager::timer_callback, this));

            if (!robot_state_client->wait_for_service(1s)) {
                RCLCPP_ERROR(this->get_logger(), "Robot state service service not available.");
                return;
            }
            if (!path_planner_client->wait_for_service(1s)) {
                RCLCPP_ERROR(this->get_logger(), "Path planner service not available.");
                return;
            }
            if (!controller_client->wait_for_action_server(1s)) {
                RCLCPP_ERROR(this->get_logger(), "Pure pursuit action service not available.");
                return;
            }
        }

    private:
        void goal_sub_callback(const geometry_msgs::msg::PoseStamped & msg) {
            goal_pose = msg;

            auto request = std::make_shared<GeneratePath::Request>();
            request->start.header.frame_id = "/map";
            // request->start.pose = odom.pose.pose;
            request->start.pose = gt_pose.pose;
            request->goal = goal_pose;

            path_planner_client->async_send_request(
                request,
                std::bind(&NavigationManager::generate_path_response_callback, this, std::placeholders::_1)
            );
        }

        void pose_callback(const geometry_msgs::msg::PoseStamped & msg) {
            gt_pose = msg;
        }

        void generate_path_response_callback(rclcpp::Client<GeneratePath>::SharedFuture future) {
            auto response = future.get();
            if (!response->success) {
                RCLCPP_WARN(get_logger(), "Path planning failed for requested goal");
                return;
            }

            path = response->path;
            path_pub->publish(path);

            FollowPath::Goal goal_msg;
            goal_msg.path = path;
            controller_client->async_send_goal(goal_msg);
        }

        void odom_callback(const nav_msgs::msg::Odometry & msg) {
            odom = msg;
        }

        void timer_callback() {
            if(path.poses.size() > 0) {
                path_pub->publish(path);
            }
        }

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        rclcpp_action::Client<warehouse_interfaces::action::FollowPath>::SharedPtr controller_client;
        rclcpp::Client<SetRobotState>::SharedPtr robot_state_client;
        rclcpp::Client<GeneratePath>::SharedPtr path_planner_client;
        rclcpp::TimerBase::SharedPtr timer;
        geometry_msgs::msg::PoseStamped goal_pose;
        geometry_msgs::msg::PoseStamped gt_pose;
        nav_msgs::msg::Path path;
        nav_msgs::msg::Odometry odom;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
