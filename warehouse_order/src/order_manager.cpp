#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <warehouse_interfaces/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

using namespace warehouse_interfaces::action;
using namespace std::chrono_literals;

class OrderManager : public rclcpp::Node {
    public:
        using NavigateToPose = warehouse_interfaces::action::NavigateToPose;
        using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        OrderManager() : Node("order_manager") {
            RCLCPP_INFO(get_logger(), "order_manager node started");
            declare_parameter("spawn_boxes", true);
            spawn_boxes_ = get_parameter("spawn_boxes").as_bool();

            order_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
                "/order",
                10,
                std::bind(&OrderManager::order_callback, this, std::placeholders::_1)
            );
            box_pub_ = create_publisher<std_msgs::msg::Int32>(
                "/spawn_box",
                10
            );
            nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

            declare_parameter<std::vector<double>>("locations_x", std::vector<double>{});
            declare_parameter<std::vector<double>>("locations_y", std::vector<double>{});
            auto locations_x = get_parameter("locations_x").as_double_array();
            auto locations_y = get_parameter("locations_y").as_double_array();
            for (size_t i = 0; i < locations_x.size() && i < locations_y.size(); ++i) {
                locations_.push_back({locations_x[i], locations_y[i]});
            }
        }
    private:
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

            if(spawn_boxes_) {
                std_msgs::msg::Int32 box_msg;
                box_msg.data = orders_[current_order_][current_item_];
                box_pub_->publish(box_msg);
                box_spawn_timer_ = create_wall_timer(2s, std::bind(&OrderManager::on_box_spawn_timer, this));
            } else {
                advance_order();
            }
        }

        void on_box_spawn_timer() {
            box_spawn_timer_->cancel();
            advance_order();
        }

        void advance_order() {
            current_item_++;

            if(current_item_ != static_cast<int>(orders_[current_order_].size())) {
                send_nav_goal(locations_[orders_[current_order_][current_item_]]);
            } else if(current_order_ != static_cast<int>(orders_.size()) - 1) {
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

        void send_nav_goal(const std::pair<double, double> & location) {
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

        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr order_sub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr box_pub_;
        rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
        rclcpp::TimerBase::SharedPtr box_spawn_timer_;
        std::vector<std::vector<int>> orders_;
        std::vector<std::pair<double, double>> locations_;
        int current_order_ = 0;
        int current_item_ = 0;
        bool active_order_ = false;
        bool spawn_boxes_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrderManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
