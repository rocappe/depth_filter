#include "depth_filter/depth_filter_node.hpp"

using std::placeholders::_1;

namespace depth_filter
{

	DepthFilter::DepthFilter() : Node("depth_filter"), k_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			got_info_{false}, point_{0.0, 0.0, 0.0}, is_bigendian_{false}, depths_{0}{
			max_height_ = this->declare_parameter("max_height", 2.0);
			min_height_ = this->declare_parameter("min_height", 0.1);
			max_distance_ = this->declare_parameter("max_distance", 5.0);
			min_distance_ = this->declare_parameter("min_distance", 0.5);
			depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>
					("depth_sub_topic", 10, std::bind(&DepthFilter::depthCallback, this, _1));
			camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>
					("color_info_topic", 10, std::bind(&DepthFilter::cameraInfoCallback, this, _1));
			depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth_pub_topic", 10);
			RCLCPP_INFO(this->get_logger(),"Node running");
	}

	void DepthFilter::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
	{
		if (!got_info_){
			RCLCPP_INFO(this->get_logger(),"Didn't get the camera info yet");
			return;
		}
		auto start = std::chrono::high_resolution_clock::now();
		uint32_t x{0};
		uint32_t y{0};

		auto img_msg = std::make_shared<sensor_msgs::msg::Image>(*msg);
		//auto img_msg = msg;
		// Get a pointer to the depth values casting the data pointer to floating point
		depths_ = (float*)(&img_msg->data[0]);
		uint32_t size = img_msg->data.size();  
		uint32_t width = img_msg->width;
		uint32_t hieght = img_msg->height;
		// multiply by 4 because the method gives the size for uint8_t not for float
		if (size != (width * hieght)*4){
			RCLCPP_WARN(this->get_logger(),"The image size is different from the declared dimensions\n"
			 "Data size: %i height*width: %i", size, img_msg->height * img_msg->width);
			return;
		}
		if (img_msg->is_bigendian == 1 && !is_bigendian_){
			RCLCPP_WARN(this->get_logger(),"The image data is bigendian");
			is_bigendian_ = true;
			exit(-1);
		}
		for (uint32_t pos = 0; pos < (width * hieght); pos++, depths_++){
			if (depths_ == 0){
				RCLCPP_WARN(this->get_logger(),"Null pointer");
				return;
			}
			else {
				if (std::isfinite(*depths_)){
					projectPoint(x, y, *depths_, k_, point_);
					float distance_2d = std::pow((std::pow(point_[0], 2.0) + std::pow(point_[2], 2.0)), 0.5);
					if (distance_2d < min_distance_ || distance_2d > max_distance_ || point_[1] > -min_height_ || point_[1]  < -max_height_){
						*depths_ = std::numeric_limits<float>::infinity();
					}
				}
			}
			if (x < width){
				x++;
			}
			else {
				x = uint32_t(0);
				y++;
			}
			//RCLCPP_WARN(this->get_logger(),"pos:%i x:%i y:%i", pos, x, y);
			}
		auto stop = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
		RCLCPP_INFO(this->get_logger(),"The filter has taken " + std::to_string(duration.count()) + " ms");
		img_msg->header.stamp = rclcpp::Time(img_msg->header.stamp) + rclcpp::Duration(duration);
		depth_pub_->publish(*img_msg);
		}

	void DepthFilter::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
	{
			if (!got_info_){
				k_ = msg->k;
				got_info_ = true;
				RCLCPP_INFO(this->get_logger(),"Camera info acquired");
			}
		}

	void DepthFilter::projectPoint(uint32_t &x, uint32_t &y, float &z, std::array<double, 9> &k_, std::array<float, 3> &point)
	{
			// It is used the optical reference frame, with z pointing out the camera,
			// looking through the camera the x axis points right and the y axis points down
			point[0] = float((double(x) - k_[2]) / k_[0] * double(z));
			point[1] = float((double(y) - k_[5])/ k_[4] * double(z));
			point[2] = z;
		}
}  // namespace depth_filter

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<depth_filter::DepthFilter>());
	rclcpp::shutdown();
	return 0;
}


