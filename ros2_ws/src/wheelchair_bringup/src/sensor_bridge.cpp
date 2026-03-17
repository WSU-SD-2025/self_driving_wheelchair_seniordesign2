#include "wheelchair_bringup/sensor_bridge.hpp"

#include <cmath>
#include <fcntl.h>
#include <sstream>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

SensorBridge::SensorBridge()
    : Node("sensor_bridge"),
      serial_fd_(-1),
      publish_tf_(false),
      left_cpr_(715.0),
      right_cpr_(1200.0),
      wheel_radius_(0.171),
      wheel_separation_(0.575),
      odom_frame_("odom"),
      base_frame_("base_link"),
      imu_frame_("imu_link"),
      odom_initialized_(false),
      prev_left_count_(0.0),
      prev_right_count_(0.0),
      prev_time_ms_(0.0),
      x_(0.0),
      y_(0.0),
      theta_(0.0)
{
    // Declare parameters
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud", 115200);
    this->declare_parameter<bool>("publish_tf", false);

    this->declare_parameter<double>("left_cpr", 715.0);
    this->declare_parameter<double>("right_cpr", 1200.0);
    this->declare_parameter<double>("wheel_radius", 0.171);
    this->declare_parameter<double>("wheel_separation", 0.575);

    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("imu_frame", "imu_link");

    // Get parameter
    port_ = this->get_parameter("port").as_string();
    baud_ = this->get_parameter("baud").as_int();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    left_cpr_ = this->get_parameter("left_cpr").as_double();
    right_cpr_ = this->get_parameter("right_cpr").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();

    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    imu_frame_ = this->get_parameter("imu_frame").as_string();

    // Create publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 20);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 50);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if(!openSerial())   throw std::runtime_error("Failed to open serial port");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&SensorBridge::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "SensorBridge node started on %s", port_.c_str());
}


bool SensorBridge::openSerial(){
    serial_fd_ = open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);

    if(serial_fd_ < 0){
        RCLCPP_ERROR(this->get_logger(), "Could not open serial port %s", port_.c_str());
        return false;
    }

    termios tty{};
    if(tcgetattr(serial_fd_, &tty) != 0){
        RCLCPP_ERROR(this->get_logger(), "tcgetattr failed");
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    cfmakeraw(&tty);

    speed_t speed = B115200;
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    if(tcsetattr(serial_fd_, TCSANOW, &tty) != 0){
        RCLCPP_ERROR(this->get_logger(), "tcsetattr failed");
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    return true;
}


bool SensorBridge::readLine(std::string& line){
    char buffer[256];
    ssize_t n = read(serial_fd_, buffer, sizeof(buffer));

    if(n > 0)   serial_buffer_.append(buffer, buffer + n);

    auto pos = serial_buffer_.find('\n');

    if(pos == std::string::npos)   return false;

    line = serial_buffer_.substr(0, pos);
    serial_buffer_.erase(0, pos + 1);

    while(!line.empty() && (line.back() == '\r' || line.back() == '\n')){
        line.pop_back();
    }

    return !line.empty();
}


std::vector<std::string> SensorBridge::split(const std::string& s, char delimiter){
    std::vector<std::string> tokens;
    std::stringstream ss(s);
    std::string item;

    while(std::getline(ss, item, delimiter)){
        tokens.push_back(item);
    }

    return tokens;
}


void SensorBridge::timerCallback(){
    std::string line;

    while(readLine(line)){
        if(line == "SENSORS_READY"){
            RCLCPP_INFO(this->get_logger(), "ESP32 sensors ready");
            continue;
        }

        if(line.rfind("ENCODER,", 0) == 0){
            handleEncoderLine(line);
        }
        else if(line.rfind("IMU,", 0) == 0){
            handleImuLine(line);
        }
    }
}


void SensorBridge::handleEncoderLine(const std::string& line){
    auto parts = split(line, ',');

    if(parts.size() != 4){
        RCLCPP_WARN(this->get_logger(), "Bad encoder line: %s", line.c_str());
        return;
    }

    try{
        double left_count = std::stod(parts[1]);
        double right_count = std::stod(parts[2]);
        double time_ms = std::stod(parts[3]);

        publishOdom(left_count, right_count, time_ms);
    }
    catch(...){
        RCLCPP_WARN(this->get_logger(), "Failed to parse encoder line: %s", line.c_str());
    }
}


void SensorBridge::handleImuLine(const std::string& line){
    auto parts = split(line, ',');

    if(parts.size() != 11){
        RCLCPP_WARN(this->get_logger(), "Bad IMU line: %s", line.c_str());
        return;
    }

    try{
        double qx = std::stod(parts[1]);
        double qy = std::stod(parts[2]);
        double qz = std::stod(parts[3]);
        double qw = std::stod(parts[4]);

        double wx = std::stod(parts[5]);
        double wy = std::stod(parts[6]);
        double wz = std::stod(parts[7]);

        double ax = std::stod(parts[8]);
        double ay = std::stod(parts[9]);
        double az = std::stod(parts[10]);

        publishImu(qx, qy, qz, qw, wx, wy, wz, ax, ay, az);
    }
    catch(...){
        RCLCPP_WARN(this->get_logger(), "Failed to parse IMU line: %s", line.c_str());
    }
}


void SensorBridge::publishOdom(double left_count, double right_count, double time_ms){
    if(!odom_initialized_){
        prev_left_count_ = left_count;
        prev_right_count_ = right_count;
        prev_time_ms_ = time_ms;
        odom_initialized_ = true;
        return;
    }

    double dt = (time_ms - prev_time_ms_) / 1000.0;

    if(dt <= 0.0){
        prev_left_count_ = left_count;
        prev_right_count_ = right_count;
        prev_time_ms_ = time_ms;
        return;
    }

    double left_delta = left_count - prev_left_count_;
    double right_delta = right_count - prev_right_count_;

    prev_left_count_ = left_count;
    prev_right_count_ = right_count;
    prev_time_ms_ = time_ms;

    double left_dist = (left_delta / left_cpr_) * (2.0 * M_PI * wheel_radius_);
    double right_dist = (right_delta / right_cpr_) * (2.0 * M_PI * wheel_radius_);

    double ds = (left_dist + right_dist) / 2.0;
    double dtheta = (right_dist - left_dist) / wheel_separation_;

    x_ += ds * std::cos(theta_ + dtheta / 2.0);
    y_ += ds * std::sin(theta_ + dtheta / 2.0);
    theta_ += dtheta;
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    double vx = ds / dt;
    double wz = dtheta / dt;

    auto now = this->get_clock()->now();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = std::sin(theta_ / 2.0);
    odom_msg.pose.pose.orientation.w = std::cos(theta_ / 2.0);

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = wz;

    odom_msg.pose.covariance = {
        0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.10, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.1
    };

    odom_msg.twist.covariance = {
        0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 99999.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.1
    };

    odom_pub_->publish(odom_msg);

    if(publish_tf_){
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = odom_frame_;
        tf_msg.child_frame_id = base_frame_;

        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;

        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = std::sin(theta_ / 2.0);
        tf_msg.transform.rotation.w = std::cos(theta_ / 2.0);

        tf_broadcaster_->sendTransform(tf_msg);
    }
}


void SensorBridge::publishImu(double qx, double qy, double qz, double qw, double wx, double wy, double wz, double ax, double ay, double az){
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = imu_frame_;

    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;
    imu_msg.orientation.w = qw;

    imu_msg.angular_velocity.x = wx;
    imu_msg.angular_velocity.y = wy;
    imu_msg.angular_velocity.z = wz;

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    imu_msg.orientation_covariance = {
        99999.0, 0.0, 0.0,
        0.0, 99999.0, 0.0,
        0.0, 0.0, 0.05
    };

    imu_msg.angular_velocity_covariance = {
        99999.0, 0.0, 0.0,
        0.0, 99999.0, 0.0,
        0.0, 0.0, 0.05
    };

    imu_msg.linear_acceleration_covariance = {
        99999.0, 0.0, 0.0,
        0.0, 99999.0, 0.0,
        0.0, 0.0, 99999.0
    };

    imu_pub_->publish(imu_msg);
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorBridge>());
    rclcpp::shutdown();
    return 0;
}