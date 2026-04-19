#include "wheelchair_bringup/sensor_bridge.hpp"
#include "wheelchair_bringup/msg/encoder_stamped.hpp"

#include <cmath>
#include <fcntl.h>
#include <sstream>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
#include <iomanip>
#include <cerrno>
#include <cstring>

SensorBridge::SensorBridge()
    : Node("sensor_bridge"),
      serial_fd_(-1),
      port_("/dev/ttyACM0"),
      baud_(115200),
      serial_ready_logged_(false),
      imu_frame_("imu_link"),
      cmd_vel_topic_("/cmd_vel"),
      encoder_topic_("/encoder/raw"),
      imu_topic_("/imu/data"),
      max_linear_(1.0),
      max_angular_(1.0),
      cmd_timeout_(0.5),
      read_period_ms_(10),
      write_period_ms_(20),
      reconnect_period_ms_(1000),
      resend_period_sec_(0.10),
      latest_linear_(0.0),
      latest_angular_(0.0),
      last_sent_linear_(9999.0),
      last_sent_angular_(9999.0),
      cmd_dirty_(true)
{
    // Declare parameters
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud", 115200);

    this->declare_parameter<std::string>("imu_frame", "imu_link");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("encoder_topic", "/encoder/raw");
    this->declare_parameter<std::string>("imu_topic", "/imu/data");

    this->declare_parameter<double>("max_linear", 1.0);
    this->declare_parameter<double>("max_angular", 1.0);
    this->declare_parameter<double>("cmd_timeout", 0.5);

    this->declare_parameter<int>("read_period_ms", 10);
    this->declare_parameter<int>("write_period_ms", 20);
    this->declare_parameter<int>("reconnect_period_ms", 1000);
    this->declare_parameter<double>("resend_period_sec", 0.10);

    // Get parameter
    port_ = this->get_parameter("port").as_string();
    baud_ = this->get_parameter("baud").as_int();

    imu_frame_ = this->get_parameter("imu_frame").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    encoder_topic_ = this->get_parameter("encoder_topic").as_string();
    imu_topic_ = this->get_parameter("imu_topic").as_string();

    max_linear_ = this->get_parameter("max_linear").as_double();
    max_angular_ = this->get_parameter("max_angular").as_double();
    cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();

    read_period_ms_ = this->get_parameter("read_period_ms").as_int();
    write_period_ms_ = this->get_parameter("write_period_ms").as_int();
    reconnect_period_ms_ = this->get_parameter("reconnect_period_ms").as_int();
    resend_period_sec_ = this->get_parameter("resend_period_sec").as_double();

    // Create publishers
    encoder_pub_ = this->create_publisher<wheelchair_bringup::msg::EncoderStamped>(encoder_topic_, 50);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 50);

    // Create Subscriber
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_,
        10,
        std::bind(&SensorBridge::cmdVelCallback, this, std::placeholders::_1)
    );

    last_cmd_time_ = this->now();
    last_send_time_ = this->now();


    // Initial serial open attempt
    if(!openSerial())   RCLCPP_WARN(this->get_logger(), "Initial serial open failed.");

    // Timers
    read_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(read_period_ms_),
        std::bind(&SensorBridge::readTimerCallback, this)
    );

    write_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(write_period_ms_),
        std::bind(&SensorBridge::writeTimerCallback, this)
    );

    reconnect_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(reconnect_period_ms_),
        std::bind(&SensorBridge::reconnectTimerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "SensorBridge node started");
    RCLCPP_INFO(this->get_logger(), "Serial port: %s @ %d baud", port_.c_str(), baud_);
    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing encoder raw to %s", encoder_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing IMU to %s", imu_topic_.c_str());
}


SensorBridge::~SensorBridge(){
    closeSerial();
}


void SensorBridge::closeSerial(){
    if(serial_fd_ >= 0){
        close(serial_fd_);
        serial_fd_ = -1;
    }
    serial_buffer_.clear();
    serial_ready_logged_ = false;
}


speed_t SensorBridge::getBaudConstant(int baud) const{
    switch(baud){
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
        default: return B115200;
    }
}


bool SensorBridge::openSerial(){
    closeSerial();

    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if(serial_fd_ < 0){
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Could not open serial port %s: %s", port_.c_str(), std::strerror(errno));
        return false;
    }

    termios tty{};
    if(tcgetattr(serial_fd_, &tty) != 0){
        RCLCPP_ERROR(this->get_logger(), "tcgetattr failed %s", std::strerror(errno));
        closeSerial();
        return false;
    }

    cfmakeraw(&tty);

    const speed_t speed = getBaudConstant(baud_);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    if(tcsetattr(serial_fd_, TCSANOW, &tty) != 0){
        RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
        closeSerial();
        return false;
    }

    tcflush(serial_fd_, TCIOFLUSH);
    
    RCLCPP_INFO(this->get_logger(), "Serial opened: %s", port_.c_str());
    return true;
}


bool SensorBridge::readLine(std::string& line){
    if(serial_fd_ < 0) return false;

    char buffer[256];
    ssize_t n = read(serial_fd_, buffer, sizeof(buffer));

    if(n > 0)   serial_buffer_.append(buffer, buffer + n);

    else if(n < 0 && errno != EAGAIN && errno != EWOULDBLOCK){
        RCLCPP_WARN(this->get_logger(), "Serial read error: %s", std::strerror(errno));
        closeSerial();
        return false;
    }

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


double SensorBridge::clamp(double x, double lo, double hi){
    return std::max(lo, std::min(x, hi));
}


void SensorBridge::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
    latest_linear_ = msg->linear.x;
    latest_angular_ = msg->angular.z;
    last_cmd_time_ = this->now();
    cmd_dirty_ = true;
}


bool SensorBridge::writeCmdPacket(double linear, double angular){
    if(serial_fd_ < 0) return false;

    std::ostringstream oss;
    oss << "<"
        << std::fixed << std::setprecision(3)
        << linear << ","
        << angular
        << ">\n";
    
    const std::string packet = oss.str();
    ssize_t n = write(serial_fd_, packet.c_str(), packet.size());

    if(n < 0){
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Failed to write to serial port: %s", std::strerror(errno));

        closeSerial();
        return false;
    }

    if(static_cast<size_t>(n) != packet.size()){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Partial serial write: %ld / %zu",
        static_cast<long>(n), packet.size());
    }
    
    return true;
}


void SensorBridge::readTimerCallback(){
    if(serial_fd_ < 0) return;

    std::string line;
    while(readLine(line)){
        if(line == "SENSORS_READY"){
            if(!serial_ready_logged_){
                RCLCPP_INFO(this->get_logger(), "ESP32 sensors ready");
                serial_ready_logged_ = true;
            }
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


void SensorBridge::writeTimerCallback(){
    if(serial_fd_ < 0) return;

    const double elapsed_cmd = (this->now() - last_cmd_time_).seconds();

    double v_cmd = 0.0;
    double w_cmd = 0.0;

    if(elapsed_cmd <= cmd_timeout_){
        v_cmd = clamp(latest_linear_, -max_linear_, max_linear_);
        w_cmd = clamp(latest_angular_, -max_angular_, max_angular_);
    }

    const double elapsed_send = (this->now() - last_send_time_).seconds();
    const bool value_changed = 
        (std::fabs(v_cmd - last_sent_linear_) > 1e-6) || (std::fabs(w_cmd - last_sent_angular_) > 1e-6);

    const bool keepalive_due = elapsed_send >= resend_period_sec_;

    if(!cmd_dirty_ && !value_changed && !keepalive_due) return;

    if(writeCmdPacket(v_cmd, w_cmd)){
        last_sent_linear_ = v_cmd;
        last_sent_angular_ = w_cmd;
        last_send_time_ = this->now();
        cmd_dirty_ = false;
    }
}


void SensorBridge::reconnectTimerCallback(){
    if(serial_fd_ >= 0) return;
    openSerial();
}


void SensorBridge::handleEncoderLine(const std::string& line){
    auto parts = split(line, ',');

    if(parts.size() != 4){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Bad encoder line: %s", line.c_str());
        return;
    }

    try{
        const int64_t left_count = std::stoll(parts[1]);
        const int64_t right_count = std::stoll(parts[2]);
        const int64_t time_ms = std::stoll(parts[3]);

        publishEncoderRaw(left_count, right_count, time_ms);
    }
    catch(...){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to parse encoder line: %s", line.c_str());
    }
}


void SensorBridge::handleImuLine(const std::string& line){
    auto parts = split(line, ',');

    if(parts.size() != 11){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Bad IMU line: %s", line.c_str());
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
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to parse IMU line: %s", line.c_str());
    }
}


void SensorBridge::publishEncoderRaw(const int64_t left_count, const int64_t right_count, const int64_t time_ms){
    
    wheelchair_bringup::msg::EncoderStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "";

    msg.left_count = left_count;
    msg.right_count = right_count;
    msg.mcu_time_ms = time_ms;

    encoder_pub_->publish(msg);
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
    imu_msg.angular_velocity.z = - wz;

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