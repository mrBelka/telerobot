#include <chrono>
#include <memory>
#include <iostream>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() : Node("camera_server"), count_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
        timer_ = this->create_wall_timer(33ms, std::bind(&MinimalPublisher::timer_callback, this));
        
        cap_.open(0);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera.");
            rclcpp::shutdown();
        }
    }

private:
    void timer_callback()
    {
        cv::Mat img;
        cap_ >> img;

        if (img.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Captured empty image.");
            return;
        }

        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();

        publisher_->publish(*msg);
        std::cout << "Published image" << std::endl;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::VideoCapture cap_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::make_shared<MinimalPublisher>();
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}