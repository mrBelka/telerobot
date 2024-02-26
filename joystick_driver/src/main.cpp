#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <ModbusMaster.hpp>
#include "joystick_msgs/msg/joystick.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Joystick : public rclcpp::Node
{
public:
    Joystick() : Node("joystick_driver")
    {
        std::string device;
        declare_parameter("dev", "/dev/ttyUSB1");
        get_parameter("dev", device);


        m_modbus = std::make_unique<robot::protocol::ModbusMaster>(device.c_str(), 115200);
        m_modbus->Setup();

        m_joystick_pub = this->create_publisher<joystick_msgs::msg::Joystick>("/joystick", 10);
        m_twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        m_joystick_timer = this->create_wall_timer(100ms, std::bind(&Joystick::joystick_callback, this));
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

private:

    double f(int x, double scale, int deadZone){
        if (-deadZone <= x and x<= deadZone) {
            return 0;
        }
        else if(x<-deadZone)
            return ((x+deadZone)*scale);
        else if(x>deadZone)
            return ((x-deadZone)*scale);
    }

    float normalize(int x, float d, int deadZone)
    {
        if (-deadZone <= x and x<= deadZone) {
            return 0;
        }
        else if(x<-deadZone)
            return float((x+deadZone))/d;
        else if(x>deadZone)
            return float((x-deadZone))/d;
    }


    void joystick_callback()
    {
        std::vector<int16_t> data = m_modbus->ReadAnalogInput(0x01, 0x0001, 14);

        auto msg = joystick_msgs::msg::Joystick();
        msg.main_forw_back = normalize(data[0] - 500, 370, 20);
        msg.main_left_right = normalize(data[1] - 520, 350, 50);
        msg.main_rotate = normalize(data[2] - 550, 300, 50);
        msg.aux_rotate = data[3];
        msg.aux_forw = data[4];
        msg.aux_back = data[5];
        msg.aux_left = data[6];
        msg.aux_right = data[7];
        msg.but_one = data[8];
        msg.but_two = data[9];
        msg.but_three = data[10];
        msg.but_four = data[11];
        msg.but_five = data[12];

        m_joystick_pub->publish(msg);

        auto msg_cmd_vel = geometry_msgs::msg::Twist();

        msg_cmd_vel.linear.x = -18.0f*normalize(data[0] - 500, 370, 20);

        m_twist_pub->publish(msg_cmd_vel);
        
    }

    std::mutex m_mutex;
    std::unique_ptr<robot::protocol::ModbusMaster> m_modbus;
    rclcpp::TimerBase::SharedPtr m_joystick_timer;
    rclcpp::Publisher<joystick_msgs::msg::Joystick>::SharedPtr m_joystick_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_twist_pub;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joystick>());
    rclcpp::shutdown();
    return 0;
}
