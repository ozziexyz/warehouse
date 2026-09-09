#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <warehouse_interfaces/action/follow_path.hpp>
#include <warehouse_interfaces/action/navigate_to_pose.hpp>
#include <warehouse_interfaces/srv/set_robot_state.hpp>
#include <warehouse_interfaces/srv/generate_path.hpp>

using namespace warehouse_interfaces::action;
using namespace warehouse_interfaces::srv;
using namespace warehouse_interfaces::msg;
using namespace std::chrono_literals;

class NavigationManager : public rclcpp::Node {
    public:
        using NavigateToPose = warehouse_interfaces::action::NavigateToPose;
        using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

        NavigationManager() : Node("navigation_manager") {
            RCLCPP_INFO(get_logger(), "navigation_manager node started");

            nav_action_server = rclcpp_action::create_server<NavigateToPose>(
                this,
                "/navigate_to_pose",
                std::bind(&NavigationManager::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&NavigationManager::handle_cancel, this, std::placeholders::_1),
                std::bind(&NavigationManager::handle_accepted, this, std::placeholders::_1)
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
            state_sub = create_subscription<RobotState>(
                "/robot_state",
                10,
                std::bind(&NavigationManager::state_callback, this, std::placeholders::_1)
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
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const NavigateToPose::Goal> goal
        ) {
            (void)uuid;
            (void)goal;
            if (state.is_moving) {
                RCLCPP_WARN(get_logger(), "Rejecting navigation goal, robot is already moving");
                return rclcpp_action::GoalResponse::REJECT;
            }
            RCLCPP_INFO(get_logger(), "Received navigation goal");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
            (void)goal_handle;
            RCLCPP_INFO(get_logger(), "Received request to cancel navigation goal");
            if (follow_path_goal_handle) {
                controller_client->async_cancel_goal(follow_path_goal_handle);
            }
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
            current_goal_handle = goal_handle;
            goal_pose = goal_handle->get_goal()->pose;

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

        void state_callback(const RobotState& msg) {
            state = msg;

            if (current_goal_handle && current_goal_handle->is_executing()) {
                auto feedback = std::make_shared<NavigateToPose::Feedback>();
                feedback->is_moving = state.is_moving;
                current_goal_handle->publish_feedback(feedback);
            }
        }

        void pose_callback(const geometry_msgs::msg::PoseStamped & msg) {
            gt_pose = msg;
        }

        void generate_path_response_callback(rclcpp::Client<GeneratePath>::SharedFuture future) {
            auto response = future.get();
            if (!response->success) {
                RCLCPP_WARN(get_logger(), "Path planning failed for requested goal");
                if (current_goal_handle && current_goal_handle->is_active()) {
                    auto result = std::make_shared<NavigateToPose::Result>();
                    result->success = false;
                    current_goal_handle->abort(result);
                }
                return;
            }

            path = response->path;
            path_pub->publish(path);

            FollowPath::Goal goal_msg;
            goal_msg.path = path;

            auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                std::bind(&NavigationManager::follow_path_goal_response_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback =
                std::bind(&NavigationManager::follow_path_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback =
                std::bind(&NavigationManager::follow_path_result_callback, this, std::placeholders::_1);

            controller_client->async_send_goal(goal_msg, send_goal_options);
        }

        void follow_path_goal_response_callback(const rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(get_logger(), "Follow path goal was rejected by the controller");
                if (current_goal_handle && current_goal_handle->is_active()) {
                    auto result = std::make_shared<NavigateToPose::Result>();
                    result->success = false;
                    current_goal_handle->abort(result);
                }
                return;
            }
            RCLCPP_INFO(get_logger(), "Follow path goal accepted by the controller");
            follow_path_goal_handle = goal_handle;

            auto request = std::make_shared<SetRobotState::Request>();
            request->state.is_moving = true;
            auto future = robot_state_client->async_send_request(request);
        }

        void follow_path_feedback_callback(
            rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr,
            const std::shared_ptr<const FollowPath::Feedback> feedback) {
            RCLCPP_DEBUG(
                get_logger(),
                "Follow path feedback: goal_distance=%.2f waypoint_distance=%.2f",
                feedback->goal_distance,
                feedback->waypoint_distance
            );
        }

        void follow_path_result_callback(const rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult & result) {
            SetRobotState::Request request;
            request.state.is_moving = false;
            auto req_ptr = std::make_shared<SetRobotState::Request>(request);
            auto future = robot_state_client->async_send_request(req_ptr);

            auto nav_result = std::make_shared<NavigateToPose::Result>();
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(get_logger(), "Follow path succeeded: %s", result.result->success ? "true" : "false");
                    nav_result->success = result.result->success;
                    if (current_goal_handle && current_goal_handle->is_active()) {
                        current_goal_handle->succeed(nav_result);
                    }
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(get_logger(), "Follow path goal was aborted");
                    nav_result->success = false;
                    if (current_goal_handle && current_goal_handle->is_active()) {
                        current_goal_handle->abort(nav_result);
                    }
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(get_logger(), "Follow path goal was canceled");
                    nav_result->success = false;
                    if (current_goal_handle && current_goal_handle->is_active()) {
                        current_goal_handle->canceled(nav_result);
                    }
                    return;
                default:
                    RCLCPP_ERROR(get_logger(), "Follow path goal ended with unknown result code");
                    nav_result->success = false;
                    if (current_goal_handle && current_goal_handle->is_active()) {
                        current_goal_handle->abort(nav_result);
                    }
                    return;
            }
        }

        void odom_callback(const nav_msgs::msg::Odometry & msg) {
            odom = msg;
        }

        void timer_callback() {
            if(path.poses.size() > 0) {
                path_pub->publish(path);
            }
        }

        rclcpp_action::Server<NavigateToPose>::SharedPtr nav_action_server;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<RobotState>::SharedPtr state_sub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        rclcpp_action::Client<warehouse_interfaces::action::FollowPath>::SharedPtr controller_client;
        rclcpp::Client<SetRobotState>::SharedPtr robot_state_client;
        rclcpp::Client<GeneratePath>::SharedPtr path_planner_client;
        rclcpp::TimerBase::SharedPtr timer;
        std::shared_ptr<GoalHandleNavigateToPose> current_goal_handle;
        rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr follow_path_goal_handle;
        geometry_msgs::msg::PoseStamped goal_pose;
        geometry_msgs::msg::PoseStamped gt_pose;
        RobotState state;
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
