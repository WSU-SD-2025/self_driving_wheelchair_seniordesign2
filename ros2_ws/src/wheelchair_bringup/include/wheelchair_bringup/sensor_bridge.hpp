#ifndef WHEELCHAIR_BRINGUP__SENSOR_BRIDGE_HPP_
#define WHEELCHAIR_BRINGUP__SENSOR_BRIDGE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>


class SensorBridge : public rclcpp::Node{
    public:
        SensorBridge();
    
    private:
        void timerCallback();

        bool openSerial();
        bool readLine(std::string& line);
        std::vector<std::string> split(const std::string& s, char delimiter);

        void handleEncoderLine(const std::string& line);
        void handleImuLine(const std::string& line);

        void publishOdom(double left_count, double right_count, double time_ms);
        void publishImu(double qx, double qy, double qz, double qw, double wx, double wy, double wz, double ax, double ay, double az);

        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void writeCmdPacket(double linear, double angular);
        double clamp(double x, double lo, double hi);


        int serial_fd_;
        std::string serial_buffer_;

        std::string port_;
        int baud_;
        bool publish_tf_;

        double left_cpr_;
        double right_cpr_;
        double wheel_radius_;
        double wheel_separation_;

        std::string odom_frame_;
        std::string base_frame_;
        std::string imu_frame_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_;

        bool odom_initialized_;
        double prev_left_count_;
        double prev_right_count_;
        double prev_time_ms_;

        double x_;
        double y_;
        double theta_;


        // cmd_vel bridge
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
        std::string cmd_vel_topic_;
        double latest_linear_;
        double latest_angular_;
        double max_linear_;
        double max_angular_;
        double cmd_timeout_;
        rclcpp::Time last_cmd_time_;
};
#endif