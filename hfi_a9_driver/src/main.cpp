#include <chrono>
#include <functional>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "telerobot_interfaces/msg/motor.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <CircularBuffer.hpp>
#include <SerialConnector.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class HfiA9_Driver : public rclcpp::Node
{
  public:
    HfiA9_Driver() : Node("hfi_a9_driver")
    {

        m_imu_orientation_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/orientation", 10);
        m_imu_accel_gyro_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/accel_gyro", 10);
        m_poll_timer = this->create_wall_timer(2ms, std::bind(&HfiA9_Driver::poll_callback, this));
		m_publish_timer = this->create_wall_timer(20ms, std::bind(&HfiA9_Driver::publish_callback, this));

        m_sc = std::make_unique<robot::io::SerialConnector>("/dev/imu", 921600);
        m_sc->Open();
    }

  private:

    void publish_callback()
    {
        
    }

    void poll_callback()
    {
        uint8_t rbuf[4096];
        size_t n = m_sc->Receive(rbuf, 4096);
        m_rx_buffer.push_many(rbuf, n);
		//for(int i=0;i<n;i++)
		//	printf("%hhX ", rbuf[i]);
		//printf("\n");
        Parser();
    }

    void Parser()
    {
        auto& buf = m_rx_buffer;
        // Parse. See .h for data frame format
        while (buf.size() >= 49 + 25)
        {
            if (buf.peek(0) == 0xAA && buf.peek(1) == 0x55 && buf.peek(2) == 0x2c)
            {
                break;
            }
            else
            {
                // discard byte:
                buf.pop();
            }
        }

        while (buf.size() >= 49 + 25)
        {
            if (buf.peek(0) == 0xAA && buf.peek(1) == 0x55 && buf.peek(2) == 0x2c)
            {
				//std::cerr << this->now().nanoseconds() << std::endl;
                std::array<uint8_t, 49> frame;
                buf.pop_many(frame.data(), frame.size());

#if 1
                const uint32_t hwTimestamp = (frame[7 + 3] << 24) |	 //
				(frame[7 + 2] << 16) |	//
				(frame[7 + 1] << 8) |  //
				(frame[7 + 0] << 0);
#endif
				//std::cerr << hwTimestamp << std::endl;

                float data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
                for (int i = 0; i < 9; i++)
                {
                    // Fixed format: little endian for this sensor.
                    uint32_t d =	//
                            (frame[11 + 4 * i + 3] << 24) |	 //
                            (frame[11 + 4 * i + 2] << 16) |	 //
                            (frame[11 + 4 * i + 1] << 8) |	//
                            (frame[11 + 4 * i + 0] << 0);
                    data[i] = *(reinterpret_cast<float*>(&d));
                }

                const float wx = data[0], wy = data[1], wz = data[2];
                const float ax = data[3], ay = data[4], az = data[5];
                const float mx = data[6], my = data[7], mz = data[8];

                /*std::cout << wx << " " << wy << " " << wz << std::endl;
                std::cout << ax << " " << ay << " " << az << std::endl;
                std::cout << mx << " " << my << " " << mz << std::endl;*/

                auto imu_msg = sensor_msgs::msg::Imu();

                imu_msg.header.frame_id = "imu";
				imu_msg.header.stamp = this->now() - rclcpp::Duration(0, 3380000); // delta between AA 55 14 pack and AA 55 2C pack

				//std::cerr << std::setprecision(14) << (this->now() - rclcpp::Duration(0, 3380000)).seconds() << std::endl;

                imu_msg.angular_velocity.x = wx;
                imu_msg.angular_velocity.y = wy;
                imu_msg.angular_velocity.z = wz;
                imu_msg.linear_acceleration.x = ax;
                imu_msg.linear_acceleration.y = ay;
                imu_msg.linear_acceleration.z = az;

                m_imu_accel_gyro_pub->publish(imu_msg);
            }
            else
            {
                buf.pop();
            }

            if (buf.peek(0) == 0xAA && buf.peek(1) == 0x55 && buf.peek(2) == 0x14)
            {
                std::array<uint8_t, 25> frame;
                buf.pop_many(frame.data(), frame.size());

#if 1 
                const uint32_t hwTimestamp = (frame[7 + 3] << 24) |	 //
				(frame[7 + 2] << 16) |	//
				(frame[7 + 1] << 8) |  //
				(frame[7 + 0] << 0);
#endif

				//std::cerr << hwTimestamp << std::endl;
				
                float data[3] = {0, 0, 0};
                for (int i = 0; i < 3; i++)
                {
                    // Fixed format: little endian for this sensor.
                    uint32_t d =	//
                            (frame[11 + 4 * i + 3] << 24) |	 //
                            (frame[11 + 4 * i + 2] << 16) |	 //
                            (frame[11 + 4 * i + 1] << 8) |	//
                            (frame[11 + 4 * i + 0] << 0);
                    data[i] = *(reinterpret_cast<float*>(&d));
                }

                float eular_div[3] = {0};
                eular_div[0] = acos(-1) * (data[0]) / 180 / 2.0;
                eular_div[1] = - acos(-1) * (data[1]) / 180 / 2.0;
                eular_div[2] = - acos(-1) * (data[2]) / 180 / 2.0;

                float ca = cos(eular_div[0]);
                float cb = cos(eular_div[1]);
                float cc = cos(eular_div[2]);
                float sa = sin(eular_div[0]);
                float sb = sin(eular_div[1]);
                float sc = sin(eular_div[2]);

                float x = sa * cb * cc - ca * sb * sc;
                float y = ca * sb * cc + sa * cb * sc;
                float z = ca * cb * sc - sa * sb * cc;
                float w = ca * cb * cc + sa * sb * sc;

                auto imu_msg = sensor_msgs::msg::Imu();



                auto orientation = geometry_msgs::msg::Quaternion();
                orientation.x = x;
                orientation.y = y;
                orientation.z = z;
                orientation.w = w;

                imu_msg.header.frame_id = "imu";
				imu_msg.header.stamp = this->now();

				//std::cerr << std::setprecision(14) << this->now().seconds() << std::endl;

                imu_msg.orientation = orientation;

                m_imu_orientation_pub->publish(imu_msg);
                //return orientation

            }
            else
            {
                // discard byte:
                buf.pop();
            }
        }
    }

	
    robot::containers::CircularBuffer<uint8_t> m_rx_buffer{4096};
    std::unique_ptr<robot::io::SerialConnector> m_sc;

    rclcpp::TimerBase::SharedPtr m_publish_timer;
    rclcpp::TimerBase::SharedPtr m_poll_timer;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_orientation_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_accel_gyro_pub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HfiA9_Driver>());
  rclcpp::shutdown();
  return 0;
}
