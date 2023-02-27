#include <chrono>
#include <functional>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

#include "telerobot_interfaces/msg/motor.hpp"
#include "telerobot_interfaces/msg/battery.hpp"
#include <ModbusMaster.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class BatteryInfo : public rclcpp::Node
{
  public:
    BatteryInfo() : Node("battery_info")
    {
    	std::string device;
    	declare_parameter("dev", ""); 
    	get_parameter("dev", device);
    	
    
  
        m_modbus = std::make_unique<robot::protocol::ModbusMaster>(device.c_str(), 9600);
        m_modbus->Setup();
        m_pub = this->create_publisher<telerobot_interfaces::msg::Battery>("battery", 10);
        m_timer = this->create_wall_timer(200ms, std::bind(&BatteryInfo::Publish, this));

    }

  private:
    void Publish()
    {
        auto msg = telerobot_interfaces::msg::Battery();

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            std::vector<uint16_t> data = m_modbus->ReadAnalogInput(0x01, 0x0001, 6);
            std::vector<float> result;
            for (int i = 0; i < data.size(); i++) {
                result.push_back((float) data[i] / 1000.0f);
            }

            /*for (auto d: result)
                RCLCPP_INFO(this->get_logger(), "%.3f", d);*/

            msg.u1 = result[0];
            msg.u2 = result[1];
            msg.u3 = result[2];
            msg.u4 = result[3];
            msg.i_load = result[4];
            msg.i_charge = result[5];

        }

        m_pub->publish(msg);
    }

    std::mutex m_mutex;
    std::unique_ptr<robot::protocol::ModbusMaster> m_modbus;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<telerobot_interfaces::msg::Battery>::SharedPtr m_pub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryInfo>());
  rclcpp::shutdown();
  return 0;
}
