#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class CmdVelDeadbandFilter : public rclcpp::Node
{
public:
  CmdVelDeadbandFilter() : Node("cmd_vel_deadband_filter")
  {
    // deadband threshold 파라미터 선언 (기본값 0.01 m/s)
    this->declare_parameter<double>("deadband_threshold", 0.01);
    deadband_threshold_ = this->get_parameter("deadband_threshold").as_double();

    // /cmd_vel 토픽 구독
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      std::bind(&CmdVelDeadbandFilter::cmdVelCallback, this, std::placeholders::_1));

    // /cmd_vel_filtered 토픽으로 필터링된 메시지 발행
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_filtered", rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "CmdVelDeadbandFilter node has been started (deadband: %.3f)", deadband_threshold_);
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto filtered_msg = geometry_msgs::msg::Twist();

    // linear.x에 deadband 적용: 임계값 이하면 0으로 설정
    if (std::abs(msg->linear.x) < deadband_threshold_) {
      filtered_msg.linear.x = 0.0;
    } else {
      filtered_msg.linear.x = msg->linear.x;
    }

    if (std::abs(msg->angular.z) < deadband_threshold_) {
        filtered_msg.angular.z = 0.0;
      } else {
        filtered_msg.angular.z = msg->angular.z;
      }

    // 나머지 값은 그대로 복사
    filtered_msg.linear.y = msg->linear.y;
    filtered_msg.linear.z = msg->linear.z;
    filtered_msg.angular.x = msg->angular.x;
    filtered_msg.angular.y = msg->angular.y;
    // filtered_msg.angular.z = msg->angular.z;

    publisher_->publish(filtered_msg);
  }

  double deadband_threshold_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelDeadbandFilter>());
  rclcpp::shutdown();
  return 0;
}
