#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <chrono>
#include <cmath>
#include <unordered_map>

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
            tag_frame_correction.setRPY(correction_roll, 0.0, 0.0);

            for(auto id : tag_ids) {
                std::string param = "tag_" + std::to_string(id) + "_pose";
                this->declare_parameter<std::vector<double>>(param, {});
                auto tf_vec = this->get_parameter(param).get_value<std::vector<double>>();
                tf2::Transform tf;
                tf2::Vector3 t(tf_vec[0], tf_vec[1], tf_vec[2]);
                tf2::Quaternion q(tf_vec[3], tf_vec[4], tf_vec[5], tf_vec[6]);
                q = tag_frame_correction * q;
                tf.setOrigin(t);
                tf.setRotation(q);
                tag_map[id] = tf;
            }

            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
            tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

            timer = create_timer(0.1s, std::bind(&AprilTagLocalization::timer_callback, this));
        }

    private:
        void timer_callback() {
            for(auto id : tag_ids) {
                std::string tag_frame = "tag36h11:" + std::to_string(id);
                geometry_msgs::msg::TransformStamped odom_to_tag;
                try {
                    odom_to_tag = tf_buffer->lookupTransform(
                    "odom", tag_frame, tf2::TimePointZero);
                } catch (const tf2::TransformException &) {
                    continue;
                }

                publish_map_odom(id, odom_to_tag);
                return;
            }
        }

        void filtered_transform(geometry_msgs::msg::TransformStamped & tf) {
            if(tf.transform.translation.z < 0) {
                tf.transform.translation.z = 0;
            }
        }

        void publish_map_odom(int id, geometry_msgs::msg::TransformStamped & tf) {
            tf2::Transform odom_tag;
            tf2::fromMsg(tf.transform, odom_tag);
            tf2::Transform map_tag = tag_map[id];
            tf2::Transform map_odom = map_tag * odom_tag.inverse();

            geometry_msgs::msg::TransformStamped out;
            out.header.stamp = this->now();
            out.header.frame_id = "map";
            out.child_frame_id = "odom";
            out.transform = tf2::toMsg(map_odom);

            filtered_transform(out);

            tf_broadcaster->sendTransform(out);
        }

        rclcpp::TimerBase::SharedPtr timer;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        std::vector<int64_t> tag_ids;
        std::unordered_map<int, tf2::Transform> tag_map;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagLocalization>());
    rclcpp::shutdown();
    return 0;
}
