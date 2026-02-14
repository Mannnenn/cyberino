#include "cybergear_driver/CyberGear.hpp"
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

enum MotorName : int
{
  LIDAR_PITCH_AXIS = 0,
};

enum MotorCanId : int
{
  LIDAR_PITCH_AXIS_CAN_ID = 0x01,
};

class CyberGearDriverNode : public rclcpp::Node
{
public:
  CyberGearDriverNode()
      : Node("cybergear_driver_node"), target_angle_(0.0f),
        min_angle_rad_(-1.57f), max_angle_rad_(1.57f) // 必要に応じて調整
  {
    // モーターリストの初期化と CyberGear のインスタンス生成
    motor_list_.push_back({LIDAR_PITCH_AXIS, LIDAR_PITCH_AXIS_CAN_ID});
    cybergear_ = new CyberGear(motor_list_, "/dev/ttyCyberGear");

    // 各モーターの初期化と有効化（モードはposition）
    for (const auto &motor : motor_list_)
    {
      int key = motor.first;
      cybergear_->init_motor(key, MODE_POSITION);
      cybergear_->set_mechpos_zero(key);
      cybergear_->set_limit_speed(key, 0.5f); // 必要に応じて調整
      cybergear_->enable_motor(key);
      RCLCPP_INFO(this->get_logger(), "Enabled motor %d", key);
    }

    // 目標角度を受信するサブスクライバ
    target_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "target_pitch_angle", 10,
        std::bind(&CyberGearDriverNode::targetCallback, this, std::placeholders::_1));

    // 現在の角度をパブリッシュするパブリッシャ
    current_pub_ = this->create_publisher<std_msgs::msg::Float32>("current_pitch_angle", 10);

    // タイマーにより定期的にモーター状態を更新してパブリッシュ
    timer_ = this->create_wall_timer(
        10ms, std::bind(&CyberGearDriverNode::updateMotor, this));
  }

  ~CyberGearDriverNode()
  {
    for (const auto &motor : motor_list_)
    {
      int key = motor.first;
      cybergear_->disable_motor(key);
      RCLCPP_INFO(this->get_logger(), "Disabled motor %d", key);
    }
    delete cybergear_;
  }

private:
  void targetCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    target_angle_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received target angle: %f", target_angle_);
  }

  void updateMotor()
  {
    // min–maxフィルタで角度を制限
    float filtered_target = std::max(min_angle_rad_, std::min(target_angle_, max_angle_rad_));

    for (const auto &motor : motor_list_)
    {
      int key = motor.first;
      cybergear_->request_status(key);
      cybergear_->set_position(key, filtered_target);
      float current = cybergear_->getPosition(key);

      auto message = std_msgs::msg::Float32();
      message.data = current;
      current_pub_->publish(message);

      RCLCPP_DEBUG(this->get_logger(), "Motor %d set to target: %f (filtered), current position: %f", key, filtered_target, current);
    }

    cybergear_->get_status();
  }

  float target_angle_;
  float min_angle_rad_; // フィルタの最小値
  float max_angle_rad_; // フィルタの最大値
  std::vector<std::pair<int, int>> motor_list_;
  CyberGear *cybergear_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CyberGearDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}