#include <chrono>
#include <memory>
#include <iostream>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("camera_client")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image", 10,
            std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

            cv::imshow("Camera Feed", img);
            cv::waitKey(1); // Обработка событий окна
        } catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriber>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}

