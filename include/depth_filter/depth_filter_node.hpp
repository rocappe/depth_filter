#ifndef DEPTH_FILTER_NODE_HPP_
#define DEPTH_FILTER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace depth_filter
{

class DepthFilter : public rclcpp::Node
{
  public:
    DepthFilter();

  private:
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void projectPoint(uint32_t &x, uint32_t &y, float &depth, std::array<double, 9> &k_, std::array<float, 3> &point);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    std::array<double, 9> k_;
    bool got_info_;
    float max_height_;
    float min_height_;
    float max_distance_;
    float min_distance_;
    std::array<float, 3> point_;
    bool is_bigendian_;
    float* depths_;
};

}  // namespace depth_filter

#endif //DEPTH_FILTER_NODE_HPP_
