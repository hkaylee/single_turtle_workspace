#include <linux/can/raw.h>
#include <linux/can.h>
#include <stdint.h>
#include "proxy/control_data.h"
#include "rclcpp/rclcpp.hpp"
#include "socketcan_interface.hpp"
#include "odrive_can.hpp"

#include "traveler_msgs/msg/odrive_status.hpp"
#include "traveler_msgs/msg/set_input_position.hpp"
#include "traveler_msgs/msg/set_state.hpp"

class can_driver : public rclcpp::Node
{
public:
    can_driver(/* args */);

    void setControl(turtle& turtle_);
    void setPosition_axis0(traveler_msgs::msg::SetInputPosition msg);
    void get_motor_status(turtle& turtle_);
    void change_odrive_state(turtle& turtle_);
    ~can_driver();

private:
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    // SocketcanInterface socket_channel0_axis0_read_;
    // SocketcanInterface socket_channel0_axis1_read_;
    // SocketcanInterface socket_channel1_axis0_read_;
    // SocketcanInterface socket_channel1_axis1_read_;
    SocketcanInterface socket_channel0_get_iq_0;
    SocketcanInterface socket_channel0_get_iq_1;
    SocketcanInterface socket_channel1_get_iq_0;
    SocketcanInterface socket_channel1_get_iq_1;
    SocketcanInterface socket_get_encoder_estimates_0_axis0;
    SocketcanInterface socket_get_encoder_estimates_0_axis1;
    SocketcanInterface socket_get_encoder_estimates_1_axis0;
    SocketcanInterface socket_get_encoder_estimates_1_axis1;
    SocketcanInterface socket_topic_set_position_0_axis0;
    SocketcanInterface socket_topic_set_position_0_axis1;
    SocketcanInterface socket_topic_set_position_1_axis3;
    SocketcanInterface socket_topic_set_position_1_axis2;
    SocketcanInterface socket_topic_set_state_0_axis0;
    SocketcanInterface socket_topic_set_state_0_axis1;
    SocketcanInterface socket_topic_set_state_1_axis3;
    SocketcanInterface socket_topic_set_state_1_axis2;
    traveler_msgs::msg::OdriveStatus odrive_status_msg_0_axis0;
    traveler_msgs::msg::OdriveStatus odrive_status_msg_0_axis1;
    traveler_msgs::msg::OdriveStatus odrive_status_msg_1_axis0;
    traveler_msgs::msg::OdriveStatus odrive_status_msg_1_axis1;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<traveler_msgs::msg::OdriveStatus>::SharedPtr publisher_0_axis0;
    rclcpp::Publisher<traveler_msgs::msg::OdriveStatus>::SharedPtr publisher_0_axis1;
    rclcpp::Publisher<traveler_msgs::msg::OdriveStatus>::SharedPtr publisher_1_axis0;
    rclcpp::Publisher<traveler_msgs::msg::OdriveStatus>::SharedPtr publisher_1_axis1;
    //void setPositionCallback(traveler_msgs::msg::SetInputPosition::SharedPtr msg);
    //rclcpp::Subscription<traveler_msgs::msg::SetInputPosition>::SharedPtr Subscription_channel_0;
    //rclcpp::Subscription<traveler_msgs::msg::SetInputPosition>::SharedPtr Subscription_channel_1;
    rclcpp::Clock ros_clock_;
    int count;
    std::chrono::system_clock::time_point now;
    //void timerCallback();
    void updateChannel2StatusCallback_0();
    void updateChannel2StatusCallback_1();
    void updateChannel1StatusCallback_0();
    void updateChannel1StatusCallback_1();
    //void setPosition_axis0(traveler_msgs::msg::SetInputPosition msg);
    void setPosition_left_adduction(traveler_msgs::msg::SetInputPosition msg);
    void setPosition_left_sweeping(traveler_msgs::msg::SetInputPosition msg);
    void setPosition_right_adduction(traveler_msgs::msg::SetInputPosition msg);
    void setPosition_right_sweeping(traveler_msgs::msg::SetInputPosition msg);
    void setstate_right_adduction(traveler_msgs::msg::SetState msg);
    void setstate_right_sweeping(traveler_msgs::msg::SetState msg);
    void setstate_left_adduction(traveler_msgs::msg::SetState msg);
    void setstate_left_sweeping(traveler_msgs::msg::SetState msg);
};