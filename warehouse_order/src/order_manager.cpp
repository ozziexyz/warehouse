#include <rclcpp/rclcpp.hpp>
#include <warehouse_interfaces/srv/set_robot_state.hpp>
#include <warehouse_interfaces/msg/robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>

using namespace warehouse_interfaces::srv;
using namespace warehouse_interfaces::msg;

class OrderManager : public rclcpp::Node {
    public:
        OrderManager() : Node("order_manager") {
            RCLCPP_INFO(get_logger(), "order_manager node started");
            state_sub_ = create_subscription<RobotState>(
                "/robot_state",
                10,
                std::bind(&OrderManager::state_callback, this, std::placeholders::_1)
            );
            goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
                "/goal",
                10
            );
            box_pub_ = create_publisher<std_msgs::msg::Int32>(
                "/spawn_box",
                10
            );

            std::vector<int> test_order = {3, 7, 10};
            orders_.push_back(test_order);

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

            timer_ = create_timer(std::chrono::seconds(1), std::bind(&OrderManager::timer_callback, this));
        }
    private:
        void state_callback(const RobotState& state) {
            state_ = state;
        }

        void timer_callback() {
            RCLCPP_INFO(get_logger(), "Is moving: %d", state_.is_moving);
            if(current_item_ < orders_[current_order_].size() && !waiting_){
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.pose.position.x = locations_[orders_[current_order_][current_item_]].first;
                pose.pose.position.y = locations_[orders_[current_order_][current_item_]].second;
                goal_pub_->publish(pose);
                waiting_ = true;
                current_item_++;
            } else if(!state_.is_moving && waiting_) {
                std_msgs::msg::Int32 loc;
                loc.data = orders_[current_order_][current_item_ - 1];
                box_pub_->publish(loc);
                waiting_ = false;
            }
        }

        rclcpp::Subscription<RobotState>::SharedPtr state_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr box_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        RobotState state_;
        std::vector<std::vector<int>> orders_;
        std::vector<std::pair<int, int>> locations_;
        int current_order_ = 0;
        int current_item_ = 0;
        bool waiting_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrderManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
