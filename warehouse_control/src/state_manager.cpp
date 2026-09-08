#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <warehouse_interfaces/srv/set_robot_state.hpp>

using namespace std::chrono_literals;

using namespace warehouse_interfaces::msg;
using namespace warehouse_interfaces::srv;

class StateManager : public rclcpp::Node {
    public:
        StateManager() : Node("state_manager") {
            RCLCPP_INFO(get_logger(), "state_manager node started");

            set_state_service = create_service<SetRobotState>(
                "/robot_state/set", 
                std::bind(&StateManager::set_state_callback, this, std::placeholders::_1, std::placeholders::_2), 
                10
            );
            robot_state_publisher = create_publisher<RobotState>("/robot_state", 10);
            publish_timer = create_timer(1s, std::bind(&StateManager::timer_callback, this));

            RobotState initial_state;
            initial_state.is_moving = false;
            state = initial_state;
        }
    
    private:
        void set_state_callback(const SetRobotState::Request::SharedPtr req, SetRobotState::Response::SharedPtr resp) {
            state = req->state;
            resp->success = true;
            RCLCPP_INFO(get_logger(), "Robot state request successful: is_moving=%d", req->state.is_moving);
        }

        void timer_callback() {
            robot_state_publisher->publish(state);
        }

        rclcpp::Service<SetRobotState>::SharedPtr set_state_service;
        rclcpp::Publisher<RobotState>::SharedPtr robot_state_publisher;
        rclcpp::TimerBase::SharedPtr publish_timer;
        RobotState state;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
