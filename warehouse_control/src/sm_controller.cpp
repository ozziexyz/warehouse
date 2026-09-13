#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <warehouse_interfaces/action/follow_path.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <mutex>

using namespace warehouse_interfaces::action;
using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

class SMController : public rclcpp::Node {
    public:
        SMController() : Node("sm_controller") {
            declare_parameter("heading_kp", 1.5);
            heading_kp_ = get_parameter("heading_kp").as_double();
            declare_parameter("heading_kt", 0.5);
            heading_kt_ = get_parameter("heading_kt").as_double();
            declare_parameter("slowdown_distance", 0.25);
            slowdown_distance_ = get_parameter("slowdown_distance").as_double();
            declare_parameter("goal_tolerance", 0.01);
            goal_tolerance_ = get_parameter("goal_tolerance").as_double();
            declare_parameter("no_turn_distance", 0.25);
            no_turn_distance_ = get_parameter("no_turn_distance").as_double();
            declare_parameter("max_drive_angle", 0.5);
            max_drive_angle_ = get_parameter("max_drive_angle").as_double();
            declare_parameter("min_turn_angle", 0.05);
            min_turn_angle_ = get_parameter("min_turn_angle").as_double();
            declare_parameter("max_v", 0.6);
            max_v_ = get_parameter("max_v").as_double();
            declare_parameter("max_w", 1.57);
            max_w_ = get_parameter("max_w").as_double();

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

            tag_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
                "/detections",
                10,
                std::bind(&SMController::tag_callback, this, std::placeholders::_1),
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

        double xte(geometry_msgs::msg::Pose j1, geometry_msgs::msg::Pose j2, geometry_msgs::msg::Pose r) {
            double path_dx = j2.position.x - j1.position.x;
            double path_dy = j2.position.y - j1.position.y;
            double path_d = hypot(path_dx, path_dy);
            double robot_dx = r.position.x - j1.position.x;
            double robot_dy = r.position.y - j1.position.y;
           
            double cross = robot_dx * path_dy - robot_dy * path_dx;
            if(path_d > 0) {
                return cross / path_d;
            } else {
                return 0;
            }
        }

        double ate(geometry_msgs::msg::Pose j1, geometry_msgs::msg::Pose j2, geometry_msgs::msg::Pose r) {
            double path_dx = j1.position.x - j2.position.x;
            double path_dy = j1.position.y - j2.position.y;
            double path_d = hypot(path_dx, path_dy);
            double robot_dx = r.position.x - j2.position.x;
            double robot_dy = r.position.y - j2.position.y;
           
            double dot = robot_dx * path_dx + robot_dy * path_dy;
            if(path_d > 0) {
                return dot / path_d;
            } else {
                return 0;
            }
        }
 
        double desired_velocity(double robot_ate) {
            double v = 0;
            if(robot_ate <= 1.5 * goal_tolerance_) {
                v = goal_tolerance_ / slowdown_distance_ * max_v_;
            } else if(robot_ate <= slowdown_distance_ && get_num_tags() == 0) {
                v = abs(robot_ate) / slowdown_distance_ * max_v_;
            } else if(robot_ate <= slowdown_distance_ && get_num_tags() > 0) {
                v = goal_tolerance_ / slowdown_distance_ * max_v_;
            } else {
                v = max_v_;
            }
            
            return v;
        }

        void tag_callback(const apriltag_msgs::msg::AprilTagDetectionArray msg) {
            std::lock_guard<std::mutex> lock(tag_mtx_);
            num_tags_ = msg.detections.size();
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(odom_mtx_);
            odom_ = msg;
        }

        nav_msgs::msg::Odometry::SharedPtr get_odom() {
            std::lock_guard<std::mutex> lock(odom_mtx_);
            return odom_;
        }

        int get_num_tags() {
            std::lock_guard<std::mutex> lock(tag_mtx_);
            return num_tags_;
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
            int next_junction = 1;

            junctions.push_back(0);
            for(int i = 1; i < (int)waypoints.size() - 1; i++) {
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
            rclcpp::Time dwell_time = get_clock()->now();
            bool dwell_timeout = false;
            
            rclcpp::Rate loop_rate(10);

            while(next_junction < (int)junctions.size()) {
                geometry_msgs::msg::Pose robot_pose = get_odom()->pose.pose;
                geometry_msgs::msg::Pose prev_junction = waypoints[junctions[next_junction-1]].pose;
                geometry_msgs::msg::Pose junction = waypoints[junctions[next_junction]].pose;

                double xt_error = xte(prev_junction, junction, robot_pose);
                double at_error = ate(prev_junction, junction, robot_pose);
                double dx = junction.position.x - robot_pose.position.x;
                double dy = junction.position.y - robot_pose.position.y;
                double d = distance(junction, robot_pose);
                double desired_heading = atan2(dy, dx);
                double heading_error = wrap_angle(desired_heading - yaw_from_quaternion(robot_pose.orientation));

                RCLCPP_INFO(get_logger(), "ATE: %f", at_error);

                geometry_msgs::msg::TwistStamped cmd_vel;
                cmd_vel.header.stamp = get_clock()->now();

                // RCLCPP_INFO(get_logger(), "Distance from junction: %f", distance(junction, robot_pose));

                if(abs(heading_error) >= max_drive_angle_ 
                    && d > no_turn_distance_ 
                    && (state_ == SMController::State::DRIVE || state_ == SMController::State::DWELL)
                ){
                    state_ = SMController::State::TURN;
                    RCLCPP_INFO(get_logger(), "State: TURN, HE: %f", heading_error);
                } else if(abs(heading_error) >= min_turn_angle_ && state_ == SMController::State::TURN) {
                    state_ = SMController::State::TURN;
                    RCLCPP_INFO(get_logger(), "State: TURN, HE: %f", heading_error);
                } else if((abs(heading_error) < min_turn_angle_ && state_ == SMController::State::TURN)
                            || (at_error < -1.5 * goal_tolerance_ && state_ == SMController::State::DRIVE && get_num_tags() == 0)
                ) {
                    dwell_time = get_clock()->now();
                    state_ = SMController::DWELL;
                    RCLCPP_INFO(get_logger(), "State: DWELL");
                } else if(state_ == SMController::State::DWELL && get_num_tags() == 0) {
                    if((get_clock()->now() - dwell_time).seconds() > 10) {
                        dwell_timeout = true;
                        break;
                    }
                    state_ = SMController::State::DWELL;
                    RCLCPP_INFO(get_logger(), "State: DWELL");
                } else {
                    state_ = SMController::State::DRIVE;
                    RCLCPP_INFO(get_logger(), "State: DRIVE");
                }

                if(state_ == SMController::State::DRIVE) {
                    // RCLCPP_INFO(get_logger(), "d: %f", d);
                    if(d < goal_tolerance_ && get_num_tags() > 0) {
                        cmd_vel.twist.linear.x = 0.0;
                        cmd_vel.twist.angular.z = 0.0;
                        next_junction++;
                    } else {
                        cmd_vel.twist.linear.x = desired_velocity(at_error);
                        if(d > no_turn_distance_) cmd_vel.twist.angular.z = heading_error * heading_kp_ + heading_kt_ * xt_error;
                    }
                } else if(state_ == SMController::State::DWELL) {
                    cmd_vel.twist.linear.x = 0.0;
                    cmd_vel.twist.angular.z = 0.0;
                } else {
                    cmd_vel.twist.linear.x = 0.0;
                    cmd_vel.twist.angular.z = heading_error * heading_kp_;
                }

                cmd_vel.twist.angular.z = std::max(std::min(cmd_vel.twist.angular.z, max_w_), -max_w_);
                cmd_vel_pub_->publish(cmd_vel);
                loop_rate.sleep();
            }

            if (rclcpp::ok() && !dwell_timeout) {
                result->success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(get_logger(), "Goal succeeded");
            } else {
                result->success = false;
                goal_handle->succeed(result);
                RCLCPP_INFO(get_logger(), "Goal failed: dwell timeout");
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
        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
        rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
        nav_msgs::msg::Odometry::SharedPtr odom_;
        std::mutex odom_mtx_;
        std::mutex tag_mtx_;
        SMController::State state_ = State::STOP;
        int num_tags_ = 0;

        double heading_kp_;
        double heading_kt_;
        double slowdown_distance_;
        double goal_tolerance_;
        double no_turn_distance_;
        double max_drive_angle_;
        double min_turn_angle_;
        double max_v_;
        double max_w_;
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