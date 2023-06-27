#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <ModbusMaster.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Joystick : public rclcpp::Node
{
public:
    Joystick() : Node("wheel_driver")
    {
        std::string device;
        declare_parameter("dev", "/dev/ttyUSB0");
        get_parameter("dev", device);


        m_modbus = std::make_unique<robot::protocol::ModbusMaster>(device.c_str(), 115200);
        m_modbus->Setup();

        m_cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        m_joystick_timer = this->create_wall_timer(1000ms, std::bind(&Joystick::joystick_callback, this));
    }

private:

    double f(int x, double scale){
        if ( -20 <= x and x<= 20) {
            return 0;
        }
        return (x*scale);
    }


    void joystick_callback()
    {
        std::vector<int16_t> data = m_modbus->ReadAnalogInput(0x01, 0x0001, 8);
        for (int i = 0; i < data.size(); i++){
            std::cout << data[i] <<std::endl;
        }
        std::cout << "=========" << std::endl;

        auto msg = geometry_msgs::msg::Twist ();

        msg.linear.x = -f(data[0],0.05);
        msg.angular.z = -f(data[1], 0.05);

        m_cmd_vel_pub->publish(msg);


        /*auto encoders_msg = telerobot_interfaces::msg::Motor();

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            std::vector<int16_t> data = m_modbus->ReadAnalogInput(0x01, 0x0001, 8);

            encoders_msg.motor_lf = (data[0]*32768) + data[1];
            encoders_msg.motor_rf = (data[2]*32768) + data[3];
            encoders_msg.motor_lr = (data[4]*32768) + data[5];
            encoders_msg.motor_rr = (data[6]*32768) + data[7];
        }
        m_encoders_pub->publish(encoders_msg);*/
    }

    std::mutex m_mutex;
    std::unique_ptr<robot::protocol::ModbusMaster> m_modbus;
    rclcpp::TimerBase::SharedPtr m_joystick_timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_pub;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joystick>());
    rclcpp::shutdown();
    return 0;
}
