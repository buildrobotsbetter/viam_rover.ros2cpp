#include <viam_rover/MotorControlNode.hpp>


// Left Motor Pins
const unsigned int IN1_PIN = 17;
const unsigned int IN2_PIN = 27;
const unsigned int ENA_PIN = 22;

const unsigned int IN3_PIN = 23; 
const unsigned int IN4_PIN = 24;
const unsigned int ENB_PIN = 25;

MotorControlNode::MotorControlNode() :
    rclcpp::Node("motor_control"),
    m_left_motor(IN1_PIN, IN2_PIN, ENA_PIN),
    m_right_motor(IN3_PIN, IN4_PIN, ENB_PIN)
{
    m_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&MotorControlNode::handle_cmd_vel, this, std::placeholders::_1));

    m_watchdog.setTimeoutInterval_ms(2000);
    m_watchdog.setTimedOutOnStart(true);
    m_watchdog.registerCallback(std::bind(&MotorControlNode::timeoutCallback, this, std::placeholders::_1));
    m_watchdog.start();
}

MotorControlNode::~MotorControlNode()
{
    m_watchdog.cancel();
}

void MotorControlNode::handle_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received cmd_vel message: linear.x = %f, angular.z = %f", msg->linear.x, msg->angular.z);

    m_watchdog.reset();

    if (msg->linear.x > 0.0)
    {
        m_left_motor.setDirection(libgpio::MotorDirection::FORWARD);
        m_right_motor.setDirection(libgpio::MotorDirection::FORWARD);

        double left_effort = 60.0;
        double right_effort = 60.0;

        RCLCPP_INFO(this->get_logger(), "Setting driver effort to: left = %f, angular.z = %f", left_effort, right_effort);

        m_left_motor.setEffort_percent(left_effort);
        m_right_motor.setEffort_percent(right_effort);
    }
    else if (msg->linear.x < 0.0)
    {
        m_left_motor.setDirection(libgpio::MotorDirection::BACKWARD);
        m_right_motor.setDirection(libgpio::MotorDirection::BACKWARD);

        double left_effort = 60.0;
        double right_effort = 60.0;

        RCLCPP_INFO(this->get_logger(), "Setting driver effort to: left = %f, angular.z = %f", left_effort, right_effort);

        m_left_motor.setEffort_percent(left_effort);
        m_right_motor.setEffort_percent(right_effort);
    }
    else if (msg->angular.z > 0.0)
    {
        m_left_motor.setDirection(libgpio::MotorDirection::BACKWARD);
        m_right_motor.setDirection(libgpio::MotorDirection::FORWARD);

        double left_effort = 50.0;
        double right_effort = 50.0;

        RCLCPP_INFO(this->get_logger(), "Setting driver effort to: left = %f, angular.z = %f", left_effort, right_effort);

        m_left_motor.setEffort_percent(left_effort);
        m_right_motor.setEffort_percent(right_effort);
    }
    else if (msg->angular.z < 0.0)
    {
        m_left_motor.setDirection(libgpio::MotorDirection::FORWARD);
        m_right_motor.setDirection(libgpio::MotorDirection::BACKWARD);

        double left_effort = 50.0;
        double right_effort = 50.0;

        RCLCPP_INFO(this->get_logger(), "Setting driver effort to: left = %f, angular.z = %f", left_effort, right_effort);

        m_left_motor.setEffort_percent(left_effort);
        m_right_motor.setEffort_percent(right_effort);
    }
    else
    {
        double left_effort = 0.0;
        double right_effort = 0.0;

        RCLCPP_INFO(this->get_logger(), "Setting driver effort to: left = %f, angular.z = %f", left_effort, right_effort);

        m_left_motor.setEffort_percent(left_effort);
        m_right_motor.setEffort_percent(right_effort);
    }
}

void MotorControlNode::timeoutCallback(Watchdog& /* watchdog */)
{
    RCLCPP_INFO(this->get_logger(), "Motor Control watchdog timed out");

    m_left_motor.setEffort_percent(0.0);
    m_right_motor.setEffort_percent(0.0);
}
