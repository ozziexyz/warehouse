#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <warehouse_interfaces/action/follow_path.hpp>
#include <warehouse_interfaces/srv/set_robot_state.hpp>

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
            path_pub = create_publisher<nav_msgs::msg::Path>("/path", 10);
            robot_state_client = create_client<SetRobotState>("/robot_state/set", 10);
            controller_client = rclcpp_action::create_client<FollowPath>(this, "/follow_path");
            timer = create_timer(0.1s, std::bind(&NavigationManager::timer_callback, this));

            if (!robot_state_client->wait_for_service(1s)) {
                RCLCPP_ERROR(this->get_logger(), "Robot state service service not available.");
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
            nav_msgs::msg::Path path = generate_path(goal_pose);
            FollowPath::Goal goal_msg;
            goal_msg.path = path;

            auto future = controller_client->async_send_goal(goal_msg);
        }

        void odom_callback(const nav_msgs::msg::Odometry & msg) {
            odom = msg;
        }

        void timer_callback() {
            
        }

        nav_msgs::msg::Path generate_path(geometry_msgs::msg::PoseStamped goal_waypoint) {
            // Creates a straight line with given pose density between current odom pose and the goal waypoint
            // TODO: with A* or similar

            geometry_msgs::msg::Pose robot_pose = odom.pose.pose;
            nav_msgs::msg::Path path;
            path.header.frame_id = "/odom";

            int pose_density = 10; // waypoints / meter
            
            double dx = goal_waypoint.pose.position.x - robot_pose.position.x;
            double dy = goal_waypoint.pose.position.y - robot_pose.position.y;
            double d = hypot(dx, dy);
            int num_poses = floor(d * pose_density);
            RCLCPP_INFO(get_logger(), "dx: %f, dy: %f", dx, dy);
            
            for(int i = 0; i < num_poses; i++) {
                geometry_msgs::msg::PoseStamped waypoint;
                waypoint.pose.position.x = (dx / (num_poses - i)) + robot_pose.position.x;
                waypoint.pose.position.y = (dy / (num_poses - i)) + robot_pose.position.y;
                RCLCPP_INFO(get_logger(), "x: %f, yy: %f", waypoint.pose.position.x, waypoint.pose.position.y);
                path.poses.push_back(waypoint);
            }

            path_pub->publish(path);
            return path;
        }

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        rclcpp_action::Client<warehouse_interfaces::action::FollowPath>::SharedPtr controller_client;
        rclcpp::Client<SetRobotState>::SharedPtr robot_state_client;
        rclcpp::TimerBase::SharedPtr timer;
        geometry_msgs::msg::PoseStamped goal_pose;
        nav_msgs::msg::Odometry odom;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
