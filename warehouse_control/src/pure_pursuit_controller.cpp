#include <cmath>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <warehouse_interfaces/action/follow_path.hpp>
#include <warehouse_interfaces/msg/robot_state.hpp>

using namespace warehouse_interfaces::action;

class PurePursuitController : public rclcpp::Node {
    public:
        using FollowPath = warehouse_interfaces::action::FollowPath;
        using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

        PurePursuitController() : Node("pure_pursuit_controller") {
            RCLCPP_INFO(get_logger(), "pure_pursuit_controller node started");

            declare_parameter<double>("lookahead_distance", 1.0);
            lookahead_distance = get_parameter("lookahead_distance").as_double();
            declare_parameter<double>("max_linear_velocity", 0.2);
            max_linear_velocity = get_parameter("max_linear_velocity").as_double();
            declare_parameter<double>("goal_tolerance", 0.1);
            goal_tolerance = get_parameter("goal_tolerance").as_double();
            declare_parameter<double>("loop_rate", 2.0);
            rate = get_parameter("loop_rate").as_double();
            declare_parameter<double>("turn_in_place_w", 1.0);
            turn_in_place_w = get_parameter("turn_in_place_w").as_double();
            declare_parameter<double>("max_turn", 70.0 * M_PI / 180.0);
            max_turn = get_parameter("max_turn").as_double();
            declare_parameter<bool>("use_ground_truth", false);
            use_ground_truth = get_parameter("use_ground_truth").as_bool();

            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            sub_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            action_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions sub_options;
            sub_options.callback_group = sub_group;

            if(use_ground_truth) {
                gt_pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/ground_truth/pose",
                    10,
                    std::bind(&PurePursuitController::gt_pose_callback, this, std::placeholders::_1),
                    sub_options
                );
            } else {
                odom_sub = create_subscription<nav_msgs::msg::Odometry>(
                    "/diff_drive_controller/odom",
                    10,
                    std::bind(&PurePursuitController::odom_callback, this, std::placeholders::_1),
                    sub_options
                );
            }
    
            action_server = rclcpp_action::create_server<FollowPath>(
                this,
                "/follow_path",
                std::bind(&PurePursuitController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&PurePursuitController::handle_cancel, this, std::placeholders::_1),
                std::bind(&PurePursuitController::handle_accepted, this, std::placeholders::_1),
                rcl_action_server_get_default_options(),
                action_group
            );
            cmd_vel_pub = create_publisher<geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", 10);
        }

    private:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(odom_mtx);
            odom = msg;
        }

        void gt_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(gt_pose_mtx);
            gt_pose = msg;
        }

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const FollowPath::Goal> goal
        ) {
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
            std::thread{std::bind(&PurePursuitController::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        double yaw_from_quaternion(geometry_msgs::msg::Quaternion q) {
            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            return atan2(siny_cosp, cosy_cosp);
        }

        nav_msgs::msg::Odometry::SharedPtr get_odom() {
            std::lock_guard<std::mutex> lock(odom_mtx);
            return odom;
        }

        geometry_msgs::msg::PoseStamped::SharedPtr get_pose() {
            std::lock_guard<std::mutex> lock(gt_pose_mtx);
            return gt_pose;
        }

        double desired_velocity(double d) {
            double scale = 1.0;
            double slowdown_distance = 10 * goal_tolerance;
            if(d <= slowdown_distance) {
                scale = d / slowdown_distance;
            }
            return max_linear_velocity * scale;
        }

        geometry_msgs::msg::PoseStamped transform_pose(
            geometry_msgs::msg::PoseStamped pose_in,
            std::string target_frame
        ) {
            geometry_msgs::msg::PoseStamped pose_out;
            try {
                pose_out = tf_buffer->transform(pose_in, target_frame);
                return pose_out;
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            }
        }

        void execute(const std::shared_ptr<GoalHandleFollowPath> goal_handle) {
            RCLCPP_INFO(get_logger(), "Executing goal");
            rclcpp::Rate loop_rate(rate);

            const auto goal = goal_handle->get_goal();
            const auto & waypoints = goal->path.poses;
            target_index = 0;

            auto feedback = std::make_shared<FollowPath::Feedback>();
            auto result = std::make_shared<FollowPath::Result>();

            double goal_distance;
            geometry_msgs::msg::Pose current_pose;

            if (use_ground_truth) {
                current_pose = get_pose()->pose;
            } else {
                current_pose = get_odom()->pose.pose;
            }

            goal_distance = distance(current_pose.position, waypoints.back().pose.position);

            while(goal_distance > goal_tolerance) {
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(get_logger(), "Goal canceled");
                    return;
                }

                if(use_ground_truth) {
                    current_pose = get_pose()->pose;
                } else {
                    current_pose = get_odom()->pose.pose;  
                }

                geometry_msgs::msg::Point robot_pos = current_pose.position;
                geometry_msgs::msg::Quaternion robot_rot = current_pose.orientation;
                geometry_msgs::msg::Point target;
            
                for (int i = target_index; i < waypoints.size(); i++) {
                    double d_new = distance(waypoints[i].pose.position, robot_pos);
                    if(d_new > lookahead_distance) {
                        // RCLCPP_INFO(get_logger(), "d_new: %f", d_new);
                        target_index = i;
                        break;
                    }
                }

                geometry_msgs::msg::PoseStamped target_pose = waypoints[target_index];
                double heading = yaw_from_quaternion(robot_rot);

                double dx = target_pose.pose.position.x - robot_pos.x;
                double dy = target_pose.pose.position.y - robot_pos.y;

                double x_local =  dx * cos(heading) + dy * sin(heading);
                double y_local = -dx * sin(heading) + dy * cos(heading);

                double target_heading = atan2(dy, dx);

                double L_actual = sqrt(x_local * x_local + y_local * y_local);
                double k = 2 * y_local / (L_actual * L_actual);
                RCLCPP_INFO(get_logger(), "k: %f, dx: %f, dy: %f, L: %f, y_local: %f, target_index: %d", k, dx, dy, L_actual, y_local, target_index);

                goal_distance = distance(robot_pos, waypoints.back().pose.position);
                feedback->waypoint_distance = distance(robot_pos, waypoints[target_index].pose.position);
                feedback->goal_distance = goal_distance;
                goal_handle->publish_feedback(feedback);

                geometry_msgs::msg::TwistStamped cmd_vel;

                double heading_error = target_heading - heading;
                heading_error = atan2(sin(heading_error), cos(heading_error));

                if(std::abs(heading_error) > max_turn) {
                    cmd_vel.twist.linear.x = 0.0;
                    cmd_vel.twist.angular.z = turn_in_place_w * (heading_error > 0) - (heading_error < 0);
                } else {
                    double linear_velocity = desired_velocity(goal_distance);
                    cmd_vel.twist.linear.x = linear_velocity;
                    cmd_vel.twist.angular.z = linear_velocity * k;
                }

                cmd_vel.header.stamp = get_clock()->now();
                cmd_vel_pub->publish(cmd_vel);

                loop_rate.sleep();
            }

            geometry_msgs::msg::TwistStamped cmd_vel;
            cmd_vel.header.stamp = get_clock()->now();
            cmd_vel_pub->publish(cmd_vel);

            if (rclcpp::ok()) {
                result->success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(get_logger(), "Goal succeeded");
            }
        }

        static float distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) {
            return static_cast<float>(std::hypot(a.x - b.x, a.y - b.y));
        }

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pose_sub;
        rclcpp_action::Server<FollowPath>::SharedPtr action_server;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub;
        rclcpp::CallbackGroup::SharedPtr sub_group, action_group;
        nav_msgs::msg::Odometry::SharedPtr odom;
        std::mutex odom_mtx;
        geometry_msgs::msg::PoseStamped::SharedPtr gt_pose;
        std::mutex gt_pose_mtx;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        int target_index = 0;
        double rate;
        double lookahead_distance;
        double max_linear_velocity;
        double goal_tolerance;
        double turn_in_place_w;
        double max_turn;
        bool use_ground_truth;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<PurePursuitController>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}