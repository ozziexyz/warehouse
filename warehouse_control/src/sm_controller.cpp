#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <warehouse_interfaces/action/follow_path.hpp>
#include <mutex>

using namespace warehouse_interfaces::action;
using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

class SMController : public rclcpp::Node {
    public:
        SMController() : Node("sm_controller") {
            sub_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            action_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions sub_options;
            sub_options.callback_group = sub_group_;

            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "/odometry/filtered",
                10,
                std::bind(&SMController::odom_callback, this, std::placeholders::_1),
                sub_options
            );

            action_server_ = rclcpp_action::create_server<FollowPath>(
                this,
                "/follow_path",
                std::bind(&SMController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&SMController::handle_cancel, this, std::placeholders::_1),
                std::bind(&SMController::handle_accepted, this, std::placeholders::_1),
                rcl_action_server_get_default_options(),
                action_group_
            );
            
            cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", 10);
        }
    private:
        double yaw_from_quaternion(geometry_msgs::msg::Quaternion q) {
            double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            return std::atan2(siny_cosp, cosy_cosp);
        }

        double distance(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2) {
            double dx = p2.position.x - p1.position.x;
            double dy = p2.position.y - p1.position.y;
            return hypot(dx, dy);
        }

        double wrap_angle(double angle) {
            return atan2(sin(angle), cos(angle));
        }
        
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(odom_mtx_);
            odom_ = msg;
        }

        nav_msgs::msg::Odometry::SharedPtr get_odom() {
            std::lock_guard<std::mutex> lock(odom_mtx_);
            return odom_;
        }

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowPath::Goal> goal) {
            (void)uuid;
            RCLCPP_INFO(get_logger(), "Received goal with %zu waypoints", goal->path.poses.size());
            if (goal->path.poses.empty()) {
                return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
            (void)goal_handle;
            RCLCPP_INFO(get_logger(), "Received request to cancel goal");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
            std::thread{std::bind(&SMController::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
            RCLCPP_INFO(get_logger(), "Executing goal");

            auto feedback = std::make_shared<FollowPath::Feedback>();
            auto result = std::make_shared<FollowPath::Result>();

            const auto goal = goal_handle->get_goal();
            const auto & waypoints = goal->path.poses;
            std::vector<int> junctions;
            int next_junction = 0;

            for(int i = 1; i < waypoints.size() - 1; i++) {
                double dpx = waypoints[i].pose.position.x - waypoints[i-1].pose.position.x;
                double dpy = waypoints[i].pose.position.y - waypoints[i-1].pose.position.y;
                double dnx = waypoints[i+1].pose.position.x - waypoints[i].pose.position.x;
                double dny = waypoints[i+1].pose.position.y - waypoints[i].pose.position.y;
                double dx = dpx - dnx;
                double dy = dpy - dny;
                if(abs(dx) > 0.01 || abs(dy) > 0.01) {
                    junctions.push_back(i);
                }
            }
            junctions.push_back(waypoints.size() - 1);

            rclcpp::Rate loop_rate(10);

            while(next_junction < junctions.size()) {
                geometry_msgs::msg::Pose robot_pose = get_odom()->pose.pose;
                geometry_msgs::msg::Pose junction = waypoints[junctions[next_junction]].pose;
                double dx = junction.position.x - robot_pose.position.x;
                double dy = junction.position.y - robot_pose.position.y;
                double desired_heading = atan2(dy, dx);
                double heading_error = wrap_angle(desired_heading - yaw_from_quaternion(robot_pose.orientation));

                geometry_msgs::msg::TwistStamped cmd_vel;
                cmd_vel.header.stamp = get_clock()->now();

                // RCLCPP_INFO(get_logger(), "Distance from junction: %f", distance(junction, robot_pose));

                if(abs(heading_error) >= 0.5 && state_ == SMController::State::DRIVE) {
                    state_ = SMController::State::TURN;
                } else if(abs(heading_error) >= 0.05 && state_ == SMController::State::TURN) {
                    state_ = SMController::State::TURN;
                } else {
                    state_ = SMController::State::DRIVE;
                }

                if(state_ == SMController::State::DRIVE) {
                    if(distance(junction, robot_pose) >= 0.1) {
                        cmd_vel.twist.linear.x = 0.75;
                        cmd_vel.twist.angular.z = heading_error * 0.6;
                    } else {
                        cmd_vel.twist.linear.x = 0.0;
                        cmd_vel.twist.angular.z = 0.0;
                        next_junction++;
                    }
                } else {
                    cmd_vel.twist.linear.x = 0.0;
                    cmd_vel.twist.angular.z = heading_error * 0.6;
                }

                cmd_vel_pub_->publish(cmd_vel);
                loop_rate.sleep();
            }

            if (rclcpp::ok()) {
                result->success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(get_logger(), "Goal succeeded");
            }
        }

        enum State {
            DRIVE,
            TURN,
            DWELL,
            STOP
        };
        rclcpp::CallbackGroup::SharedPtr sub_group_, action_group_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
        nav_msgs::msg::Odometry::SharedPtr odom_;
        std::mutex odom_mtx_;
        SMController::State state_ = State::STOP;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<SMController>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}