#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <chrono>
#include <cmath>
#include <unordered_map>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

using namespace std::chrono_literals;

class AprilTagLocalization : public rclcpp::Node {
    public:
        AprilTagLocalization() : Node("apriltag_localization") {
            RCLCPP_INFO(get_logger(), "apriltag_localization node started");

            declare_parameter<std::vector<int64_t>>("tag_ids", {});
            tag_ids = this->get_parameter("tag_ids").as_integer_array();

            // The tag poses in tag_*_pose are derived from each tag model's box geometry in
            // warehouse.sdf, but apriltag_ros's detected tag frame is defined by the printed
            // pattern instead, which is rotated relative to that box. This correction was
            // found empirically by comparing a detected tag's tf to its configured pose.
            declare_parameter<double>("tag_frame_correction_roll", M_PI_2);
            double correction_roll = this->get_parameter("tag_frame_correction_roll").as_double();
            tf2::Quaternion tag_frame_correction;
            tag_frame_correction.setRPY(0.0, 0.0, -M_PI_2);

            for(auto id : tag_ids) {
                std::string param = "tag_" + std::to_string(id) + "_pose";
                this->declare_parameter<std::vector<double>>(param, {});
                auto tf_vec = this->get_parameter(param).get_value<std::vector<double>>();
                tf2::Transform tf;
                tf2::Vector3 t(tf_vec[0], tf_vec[1], tf_vec[2]);
                tf2::Quaternion q(tf_vec[3], tf_vec[4], tf_vec[5], tf_vec[6]);
                q = q * tag_frame_correction;
                tf.setOrigin(t);
                tf.setRotation(q);
                tag_map[id] = tf;
            }

            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            detection_sub = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
                "/detections",
                10,
                std::bind(&AprilTagLocalization::detection_callback, this, std::placeholders::_1)
            );
            timer = create_timer(0.1s, std::bind(&AprilTagLocalization::timer_callback, this));
            pose_pub = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/apriltag/pose", 10);
        }

    private:
        void timer_callback() {
            for(auto tag : detections.detections) {
                std::string tag_frame = "tag36h11:" + std::to_string(tag.id);
                geometry_msgs::msg::TransformStamped odom_to_tag;
                try {
                    odom_to_tag = tf_buffer->lookupTransform(
                    "base_footprint", tag_frame, tf2::TimePointZero);
                } catch (const tf2::TransformException &) {
                    continue;
                }
                publish_pose_estimate(tag.id, odom_to_tag);
                return;
            }
        }

        void filtered_transform(geometry_msgs::msg::TransformStamped & tf) {
            if(tf.transform.translation.z < 0) {
                tf.transform.translation.z = 0;
            }
        }

        void detection_callback(const apriltag_msgs::msg::AprilTagDetectionArray& msg) {
            detections = msg;
        }

        std::array<double, 36UL> get_covarince(double distance) {
            double d_scale = 0.05;
            double base_xy = 0.02;
            double base_yaw = 0.01;
            double d_power = 2.0;
            double max_xy = 2.0;
            double max_yaw = 1.5;

            double dist_term = d_scale * pow(distance, d_power);
            double stddev_xy = std::min(base_xy + dist_term, max_xy);
            double stddev_yaw = std::min(base_yaw + 0.5 * dist_term, max_yaw);

            double var_xy = pow(stddev_xy, 2);
            double var_yaw = pow(stddev_yaw, 2);

            std::array<double, 36UL> cov = {
                var_xy, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, var_xy, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1E6, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1E6, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1E6, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, var_yaw
            };

            return cov;
        }

        void publish_pose_estimate(int id, geometry_msgs::msg::TransformStamped & tf) {
            tf2::Transform base_tag;
            tf2::fromMsg(tf.transform, base_tag);
            tf2::Transform map_tag = tag_map[id];
            tf2::Transform map_base = map_tag * base_tag.inverse();
            // filtered_transform(out);

            double distance = sqrt(
                pow(tf.transform.translation.x, 2) + 
                pow(tf.transform.translation.y, 2) + 
                pow(tf.transform.translation.z, 2)
            );

            geometry_msgs::msg::Transform map_base_msg = tf2::toMsg(map_base);
            geometry_msgs::msg::PoseWithCovarianceStamped out;
            out.header.stamp = tf.header.stamp;
            out.header.frame_id = "map";
            out.pose.pose.position.x = map_base_msg.translation.x;
            out.pose.pose.position.y = map_base_msg.translation.y;
            out.pose.pose.position.z = map_base_msg.translation.z;
            out.pose.pose.orientation = map_base_msg.rotation;
            out.pose.covariance = get_covarince(distance);

            pose_pub->publish(out);
        }

        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;
        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        apriltag_msgs::msg::AprilTagDetectionArray detections;
        std::vector<int64_t> tag_ids;
        std::unordered_map<int, tf2::Transform> tag_map;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagLocalization>());
    rclcpp::shutdown();
    return 0;
}
