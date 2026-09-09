#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <warehouse_interfaces/srv/set_robot_state.hpp>
#include <warehouse_interfaces/msg/robot_state.hpp>
#include <warehouse_interfaces/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

using namespace warehouse_interfaces::srv;
using namespace warehouse_interfaces::msg;
using namespace warehouse_interfaces::action;

class OrderManager : public rclcpp::Node {
    public:
        using NavigateToPose = warehouse_interfaces::action::NavigateToPose;
        using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        OrderManager() : Node("order_manager") {
            RCLCPP_INFO(get_logger(), "order_manager node started");
            state_sub_ = create_subscription<RobotState>(
                "/robot_state",
                10,
                std::bind(&OrderManager::state_callback, this, std::placeholders::_1)
            );
            order_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
                "/order",
                10,
                std::bind(&OrderManager::order_callback, this, std::placeholders::_1)
            );
            goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
                "/goal",
                10
            );
            box_pub_ = create_publisher<std_msgs::msg::Int32>(
                "/spawn_box",
                10
            );
            nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

            locations_.push_back({-1, 4});
            locations_.push_back({0, 4});
            locations_.push_back({1, 4});

            locations_.push_back({-1, 2});
            locations_.push_back({0, 2});
            locations_.push_back({1, 2});

            locations_.push_back({-1, 0});
            locations_.push_back({0, 0});
            locations_.push_back({1, 0});

            locations_.push_back({-1, -2});
            locations_.push_back({0, -2});
            locations_.push_back({1, -2});

            locations_.push_back({-1, -4});
            locations_.push_back({0, -4});
            locations_.push_back({1, -4});

        }
    private:
        void state_callback(const RobotState& state) {
            state_ = state;
        }

        void order_callback(const std_msgs::msg::Int32MultiArray& msg) {
            orders_.push_back(msg.data);
            if(!active_order_) {
                handle_order();
            }
        }

        void nav_goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(get_logger(), "Navigate to pose goal was rejected by the navigation manager");
                return;
            }
            RCLCPP_INFO(get_logger(), "Navigate to pose goal accepted by the navigation manager");
        }

        void nav_feedback_callback(
            GoalHandleNavigateToPose::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
            RCLCPP_DEBUG(get_logger(), "Navigate to pose feedback: is_moving=%s", feedback->is_moving ? "true" : "false");
        }

        void nav_result_callback(const GoalHandleNavigateToPose::WrappedResult & result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(get_logger(), "Navigate to pose succeeded: %s", result.result->success ? "true" : "false");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(get_logger(), "Navigate to pose goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(get_logger(), "Navigate to pose goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(get_logger(), "Navigate to pose goal ended with unknown result code");
                    break;
            }

            current_item_++;

            if(current_item_ != orders_[current_order_].size()) {
                send_nav_goal(locations_[orders_[current_order_][current_item_]]);
            } else if(current_order_ != orders_.size() - 1) {
                current_order_++;
                current_item_ = 0;
                handle_order();
            } else {
                current_item_ = 0;
                current_order_++;
                active_order_ = false;
            }
        }

        void handle_order() {
            active_order_ = true;
            send_nav_goal(locations_[orders_[current_order_][current_item_]]);
        }

        void send_nav_goal(const std::pair<int, int> & location) {
            NavigateToPose::Goal goal;
            goal.pose.pose.position.x = location.first;
            goal.pose.pose.position.y = location.second;

            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                std::bind(&OrderManager::nav_goal_response_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback =
                std::bind(&OrderManager::nav_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback =
                std::bind(&OrderManager::nav_result_callback, this, std::placeholders::_1);

            nav_action_client_->async_send_goal(goal, send_goal_options);
        }

        rclcpp::Subscription<RobotState>::SharedPtr state_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr order_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr box_pub_;
        rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
        RobotState state_;
        std::vector<std::vector<int>> orders_;
        std::vector<std::pair<int, int>> locations_;
        int current_order_ = 0;
        int current_item_ = 0;
        bool active_order_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrderManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
