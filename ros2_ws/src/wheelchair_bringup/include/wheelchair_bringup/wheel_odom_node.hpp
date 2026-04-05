#ifndef WHEELCHAIR_BRINGUP__WHEEL_ODOM_NODE_HPP_
#define WHEELCHAIR_BRINGUP__WHEEL_ODOM_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class WheelOdomNode : public rclcpp::Node{
    public:
        WheelOdomNode();

    private:
        void encoderCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg);
        double wrapAngle(double a);

        // Params
        double left_cpr_;
        double right_cpr_;
        double wheel_radius_;
        double wheel_separation_;

        std::string odom_frame_;
        std::string base_frame_;

        std::string encoder_topic_;
        std::string odom_topic_;

        bool publish_tf_;

        // State
        bool initialized_;

        int64_t prev_left_count_;
        int64_t prev_right_count_;
        int64_t prev_time_ms_;

        double x_;
        double y_;
        double theta_;

        // ROS
        rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr encoder_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
#endif