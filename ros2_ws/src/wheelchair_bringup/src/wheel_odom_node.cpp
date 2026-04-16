#include "wheelchair_bringup/wheel_odom_node.hpp"

#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <cstdlib>

WheelOdomNode::WheelOdomNode()
    : Node("wheel_odom_node"),
      left_cpr_(715.0),
      right_cpr_(1200.0),
      wheel_radius_(0.171),
      wheel_separation_(0.575),
      odom_frame_("odom"),
      base_frame_("base_link"),
      encoder_topic_("/encoder/raw"),
      odom_topic_("/wheel/odom"),
      publish_tf_(true),
      initialized_(false),
      prev_left_count_(0),
      prev_right_count_(0),
      prev_time_ms_(0),
      x_(0.0),
      y_(0.0),
      theta_(0.0)
{
    this->declare_parameter<double>("left_cpr", 715.0);
    this->declare_parameter<double>("right_cpr", 1200.0);
    this->declare_parameter<double>("wheel_radius", 0.171);
    this->declare_parameter<double>("wheel_separation", 0.575);

    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("encoder_topic", "/encoder/raw");
    this->declare_parameter<std::string>("odom_topic", "/wheel/odom");
    this->declare_parameter<bool>("publish_tf", true);

    left_cpr_ = this->get_parameter("left_cpr").as_double();
    right_cpr_ = this->get_parameter("right_cpr").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();

    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    encoder_topic_ = this->get_parameter("encoder_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 50);
    encoder_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
        encoder_topic_, 50,
        std::bind(&WheelOdomNode::encoderCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "WheelOdomNode started");
}

double WheelOdomNode::wrapAngle(double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}

void WheelOdomNode::encoderCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 3) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Encoder raw message size < 3");
        return;
    }

    const int64_t left_count = msg->data[0];
    const int64_t right_count = msg->data[1];
    const int64_t time_ms = msg->data[2];

    if (!initialized_) {
        prev_left_count_ = left_count;
        prev_right_count_ = right_count;
        prev_time_ms_ = time_ms;
        initialized_ = true;
        return;
    }

    const double dt = static_cast<double>(time_ms - prev_time_ms_) / 1000.0;
    if (dt <= 1e-4) {
        return;
    }

    const int64_t left_delta = left_count - prev_left_count_;
    const int64_t right_delta = right_count - prev_right_count_;

    prev_left_count_ = left_count;
    prev_right_count_ = right_count;
    prev_time_ms_ = time_ms;

    const int64_t left_delta_used = (std::llabs(left_delta) <= 1) ? 0 : left_delta;
    const int64_t right_delta_used = (std::llabs(right_delta) <= 1) ? 0 : right_delta;

    const double left_dist =
        (static_cast<double>(left_delta_used) / left_cpr_) * (2.0 * M_PI * wheel_radius_);
    const double right_dist =
        (static_cast<double>(right_delta_used) / right_cpr_) * (2.0 * M_PI * wheel_radius_);

    double ds = (left_dist + right_dist) / 2.0;
    double dtheta = (right_dist - left_dist) / wheel_separation_;

    if (std::fabs(ds) < 1e-4) {
        ds = 0.0;
    }
    if (std::fabs(dtheta) < 1e-4) {
        dtheta = 0.0;
    }

    x_ += ds * std::cos(theta_ + dtheta / 2.0);
    y_ += ds * std::sin(theta_ + dtheta / 2.0);
    theta_ = wrapAngle(theta_ + dtheta);

    double vx = ds / dt;
    double wz = dtheta / dt;

    if (std::fabs(vx) > 2.0) {
        vx = 0.0;
    }
    if (std::fabs(wz) > 2.5) {
        wz = 0.0;
    }

    const auto now = this->now();

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

    if (publish_tf_) {
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdomNode>());
    rclcpp::shutdown();
    return 0;
}