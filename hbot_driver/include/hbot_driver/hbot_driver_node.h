#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "serial/serial.h"
#include "hbot_msg/msg/rpm.hpp"

constexpr int REC_UART_BUFFER_SIZE = 12;

using namespace std::chrono_literals;
using std::placeholders::_1;


class HbotDriver : public rclcpp::Node
{
private:
  void getParams();
  void readUart();
  void SendCommand();
  bool connectStm32();
  void setRpmCallback(const hbot_msg::msg::Rpm::SharedPtr msg);

  rclcpp::Publisher<hbot_msg::msg::Rpm>::SharedPtr fb_rpm_pub_;
  rclcpp::Subscription<hbot_msg::msg::Rpm>::SharedPtr rpm_set_sub_;

  int baudrate_;
  std::string port_;
  serial::Serial* serial_;
  rclcpp::TimerBase::SharedPtr timer_receive_uart_;
  uint32_t rec_count_ = 0;

  int16_t fb_rpm_left_, fb_rpm_right_;
  int16_t set_rpm_left_, set_rpm_right_;
  int16_t target_rpm_left_, target_rpm_right_;
public:
  HbotDriver();
  ~HbotDriver();
};
