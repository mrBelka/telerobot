#include <chrono>
#include <functional>
#include <memory>

#include "retrans.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "telerobot_interfaces/msg/motor.hpp"
#include "std_msgs/msg/string.hpp"
#include "telerobot_interfaces/msg/motor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


#define TICK_TO_RAD 0.00253354246

using std::placeholders::_1;

class Odometry : public rclcpp::Node
{
  public:
    Odometry() : Node("telebot_odometry")
    {
        m_encodes_sub = this->create_subscription<telerobot_interfaces::msg::Motor>(
            "encodes", 10, std::bind(&Odometry::encodes_callback, this, _1));

        m_joint_states_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        m_last_time = this->now();

        odom_class = std::make_unique<OdometryClass>(4, 10, 10);
        m_pub = this->create_publisher<telerobot_interfaces::msg::Motor>("wheel_commands", 10);
        m_odom_commands_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&Odometry::odom_commands, this, _1));
    }

  private:

    void encodes_callback(const telerobot_interfaces::msg::Motor& encodes_msg)
    {
        rclcpp::Time now = this->now();
        double motor_lf_position = TICK_TO_RAD * encodes_msg.motor_lf;
        double motor_rf_position = TICK_TO_RAD * encodes_msg.motor_lr;
        double motor_lr_position = TICK_TO_RAD * encodes_msg.motor_rf;
        double motor_rr_position = TICK_TO_RAD * encodes_msg.motor_rr;

        // Joint states block
        auto joint_states_msg = std::make_unique<sensor_msgs::msg::JointState>();
        joint_states_msg->header.frame_id = "base_link";
        joint_states_msg->header.stamp = now;
        joint_states_msg->name.emplace_back("front_left_wheel_joint");
        joint_states_msg->name.emplace_back("front_right_wheel_joint");
        joint_states_msg->name.emplace_back("rear_left_wheel_joint");
        joint_states_msg->name.emplace_back("rear_right_wheel_joint");

        joint_states_msg->position.push_back(motor_lf_position);
        joint_states_msg->position.push_back(motor_rf_position);
        joint_states_msg->position.push_back(motor_lr_position);
        joint_states_msg->position.push_back(motor_rr_position);

        m_joint_states_pub->publish(std::move(joint_states_msg));

        // Odometry block
        if(m_first)
        {
            m_last_joint_positions[0] = motor_lf_position;
            m_last_joint_positions[1] = motor_rf_position;
            m_last_joint_positions[2] = motor_lr_position;
            m_last_joint_positions[3] = motor_rr_position;
            m_first = false;
            return;
        }
        m_diff_joint_positions[0] = motor_lf_position - m_last_joint_positions[0];
        m_diff_joint_positions[1] = motor_rf_position - m_last_joint_positions[1];
        m_diff_joint_positions[2] = motor_lr_position - m_last_joint_positions[2];
        m_diff_joint_positions[3] = motor_rr_position - m_last_joint_positions[3];
        m_last_joint_positions[0] = motor_lf_position;
        m_last_joint_positions[1] = motor_rf_position;
        m_last_joint_positions[2] = motor_lr_position;
        m_last_joint_positions[3] = motor_rr_position;



        rclcpp::Duration duration(rclcpp::Duration::from_nanoseconds(
                now.nanoseconds() - m_last_time.nanoseconds()));
        double step_time = duration.seconds();

        double wheel_lf = m_diff_joint_positions[0];
        double wheel_rf = m_diff_joint_positions[1];
        double wheel_lr = m_diff_joint_positions[2];
        double wheel_rr = m_diff_joint_positions[3];

        FORWARD_DATA forw_data {wheel_lf, wheel_rf, wheel_lr, wheel_rr};

        odom_class->Update(step_time, forw_data);

        RCLCPP_INFO(this->get_logger(), "%f", odom_class->posx);
        RCLCPP_INFO(this->get_logger(), "%f", odom_class->posy);
        RCLCPP_INFO(this->get_logger(), "%f", odom_class->rot);


        m_last_time = this->now();

    }


    void odom_commands(const geometry_msgs::msg::Twist & msg)
    {
        
        RCLCPP_INFO(this->get_logger(), "%f", msg.linear.x);
        RCLCPP_INFO(this->get_logger(), "%f", msg.linear.y);
        RCLCPP_INFO(this->get_logger(), "%f", msg.angular.z);

        auto msg1 = telerobot_interfaces::msg::Motor();

        {
            INVERSE_DATA inv_data{msg.linear.x, msg.linear.y, msg.angular.z};
            FORWARD_DATA data = odom_class->GetOdometryINV(inv_data);
            //std::lock_guard<std::mutex> lock(m_mutex);

            msg1.motor_lf = data.WFL * 100;
            msg1.motor_rf = data.WFR * 100;
            msg1.motor_lr = data.WRL * 100;
            msg1.motor_rr = data.WRR * 100;
        }

        m_pub->publish(msg1);

    }

    rclcpp::Subscription<telerobot_interfaces::msg::Motor>::SharedPtr m_encodes_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_states_pub;

    double m_diff_joint_positions[4] = {0};
    double m_last_joint_positions[4] = {0};
    bool m_first = true;

    rclcpp::Time m_last_time;
    std::unique_ptr<OdometryClass> odom_class;
    rclcpp::Publisher<telerobot_interfaces::msg::Motor>::SharedPtr m_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_odom_commands_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
