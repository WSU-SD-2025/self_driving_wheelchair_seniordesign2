#ifndef WHEELCHAIR_BRINGUP__SENSOR_BRIDGE_HPP_
#define WHEELCHAIR_BRINGUP__SENSOR_BRIDGE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>


class SensorBridge : public rclcpp::Node{
    public:
        SensorBridge();
        ~SensorBridge();
    
    private:
        // Timers
        void readTimerCallback();
        void writeTimerCallback();
        void reconnectTimerCallback();

        // Serial
        bool openSerial();
        void closeSerial();
        bool readLine(std::string& line);
        bool writeCmdPacket(double linear, double angular);
        speed_t getBaudConstant(int baud) const;

        // Parsing
        std::vector<std::string> split(const std::string& s, char delimiter);
        void handleEncoderLine(const std::string& line);
        void handleImuLine(const std::string& line);

        // Publish
        void publishEncoderRaw(int64_t left_count, int64_t right_count, int64_t time_ms);
        void publishImu(double qx, double qy, double qz, double qw, double wx, double wy, double wz, double ax, double ay, double az);

        // cmd_vel
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        double clamp(double x, double lo, double hi);

        // Serial state
        int serial_fd_;
        std::string serial_buffer_;
        std::string port_;
        int baud_;
        bool serial_ready_logged_;

        // Topics/ Params
        std::string imu_frame_;
        std::string cmd_vel_topic_;
        std::string encoder_topic_;
        std::string imu_topic_;

        double max_linear_;
        double max_angular_;
        double cmd_timeout_;

        int read_period_ms_;
        int write_period_ms_;
        int reconnect_period_ms_;
        double resend_period_sec_;

        // cmd state
        double latest_linear_;
        double latest_angular_;
        double last_sent_linear_;
        double last_sent_angular_;
        bool cmd_dirty_;

        rclcpp::Time last_cmd_time_;
        rclcpp::Time last_send_time_;

        // ROS2 interfaces
        rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr encoder_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

        rclcpp::TimerBase::SharedPtr read_timer_;
        rclcpp::TimerBase::SharedPtr write_timer_;
        rclcpp::TimerBase::SharedPtr reconnect_timer_;
};
#endif