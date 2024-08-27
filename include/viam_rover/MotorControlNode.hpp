#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <libgpio/MotorDriver.hpp>
#include <utilities/Watchdog.hpp>


class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode();
    ~MotorControlNode() override;

private:
    void handle_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void timeoutCallback(Watchdog& watchdog);
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_sub;

    libgpio::MotorDriver m_left_motor;
    libgpio::MotorDriver m_right_motor;
    Watchdog m_watchdog;
};