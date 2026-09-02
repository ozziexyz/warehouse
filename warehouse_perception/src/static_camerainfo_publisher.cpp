#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class StaticCameraInfoPublisher : public rclcpp::Node {
    public:
        StaticCameraInfoPublisher() : Node("static_camerainfo_publisher") {
            RCLCPP_INFO(get_logger(), "static_camerainfo_publisher node started");

            image_sub = create_subscription<sensor_msgs::msg::Image>(
                "/image_raw",
                10,
                std::bind(&StaticCameraInfoPublisher::image_callback, this, std::placeholders::_1)
            );
            info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(
                "/camera_info",
                10
            );

            info.width = 640;
            info.height = 480;

            double hfov = 1.4;
            double fx = info.width / (2 * tan(hfov / 2));
            double fy = fx;
            double cx = info.width / 2.0;
            double cy = info.height / 2.0;

            info.distortion_model = "plumb_bob";
            info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
            info.k = {fx, 0.0, cx,  0.0, fy, cy,  0.0, 0.0, 1.0};
            info.r = {1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0};
            info.p = {fx, 0.0, cx, 0.0,  0.0, fy, cy, 0.0,  0.0, 0.0, 1.0, 0.0};
        }
    
    private:
        void image_callback(sensor_msgs::msg::Image msg) {
            info.header = msg.header;
            info_pub->publish(info);
        }
        
        sensor_msgs::msg::CameraInfo info;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StaticCameraInfoPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
