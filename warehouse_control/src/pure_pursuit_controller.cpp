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

using namespace warehouse_interfaces::action;

class PurePursuitController : public rclcpp::Node {
    public:
        using FollowPath = warehouse_interfaces::action::FollowPath;
        using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

        PurePursuitController() : Node("pure_pursuit_controller") {
            declare_parameter<double>("lookahead_distance", 1.0);
            lookahead_distance = get_parameter("lookahead_distance").as_double();

            sub_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            action_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            rclcpp::SubscriptionOptions sub_options;
            sub_options.callback_group = sub_group;

            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            odom_sub = create_subscription<nav_msgs::msg::Odometry>(
                "/diff_drive_controller/odom",
                10,
                std::bind(&PurePursuitController::odom_callback, this, std::placeholders::_1),
                sub_options
            );

            action_server = rclcpp_action::create_server<FollowPath>(
                this,
                "follow_path",
                std::bind(&PurePursuitController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&PurePursuitController::handle_cancel, this, std::placeholders::_1),
                std::bind(&PurePursuitController::handle_accepted, this, std::placeholders::_1),
                rcl_action_server_get_default_options(),
                action_group
            );

            cmd_vel_pub = create_publisher<geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", 10);

            RCLCPP_INFO(get_logger(), "pure_pursuit_controller node started");
        }

    private:
        void odom_callback(const nav_msgs::msg::Odometry & msg) {
            std::lock_guard<std::mutex> lock(odom_mtx);
            odom = msg;
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
            rclcpp::Rate loop_rate(2);

            const auto goal = goal_handle->get_goal();
            const auto & waypoints = goal->path.poses;

            auto feedback = std::make_shared<FollowPath::Feedback>();
            auto result = std::make_shared<FollowPath::Result>();

            while(target_index <= waypoints.size() - 1) {
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(get_logger(), "Goal canceled");
                    return;
                }

                nav_msgs::msg::Odometry current_odom;
                {
                    std::lock_guard<std::mutex> lock(odom_mtx);
                    current_odom = odom;
                }

                geometry_msgs::msg::Point robot_pos = current_odom.pose.pose.position;
                geometry_msgs::msg::Quaternion robot_rot = current_odom.pose.pose.orientation;
                geometry_msgs::msg::Point target;
            
                for (int i = last_index; i < waypoints.size(); i++) {
                    double d_new = distance(waypoints[i].pose.position, robot_pos);
                    double d_last = distance(waypoints[i].pose.position, waypoints[last_index].pose.position);
                    if(d_new > lookahead_distance && d_new > d_last) {
                        last_index = target_index;      
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

                double  L_actual = sqrt(x_local * x_local + y_local * y_local);
                double k = 2 * y_local / (L_actual * L_actual);
                RCLCPP_INFO(get_logger(), "k: %f", k);

                feedback->waypoint_distance = distance(robot_pos, waypoints[target_index].pose.position);
                feedback->goal_distance = distance(robot_pos, waypoints.back().pose.position);
                goal_handle->publish_feedback(feedback);

                geometry_msgs::msg::TwistStamped cmd_vel;
                cmd_vel.twist.angular.z = linear_velocity * k;
                cmd_vel.twist.linear.x = linear_velocity;

                cmd_vel_pub->publish(cmd_vel);

                loop_rate.sleep();
            }

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
        rclcpp_action::Server<FollowPath>::SharedPtr action_server;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub;
        rclcpp::CallbackGroup::SharedPtr sub_group, action_group;
        nav_msgs::msg::Odometry odom;
        std::mutex odom_mtx;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        int target_index = 0;
        int last_index = 0;
        double lookahead_distance;
        double linear_velocity = 0.2;
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
