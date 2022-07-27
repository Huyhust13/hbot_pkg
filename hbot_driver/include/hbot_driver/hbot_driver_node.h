#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "serial/serial.h"
#include "hbot_msg/msg/rpm.hpp"

constexpr int REC_UART_BUFFER_SIZE = 12;

using namespace std::chrono_literals;
using std::placeholders::_1;

constexpr uint8_t CMD_SET_VELOCITY = 0x50;
constexpr uint8_t CMD_SET_KP = 0x55;
constexpr uint8_t CMD_SET_KI = 0x56;
constexpr uint8_t CMD_SET_KD = 0x57;

constexpr uint8_t KEY_BEGIN_TRAN = 0x2A;
constexpr uint8_t KEY_END_TRAN = 0x3E;

constexpr uint8_t KEY_BEGIN_REC = 0x2B;
constexpr uint8_t KEY_END_REC = 0x3F;

constexpr uint8_t HEADER_SIZE = 0x06;
class HbotDriver : public rclcpp::Node
{
private:
  void getParams();
  bool connectStm32();
  void readUart();
  bool sendCommand(std::vector<uint8_t> buff, uint8_t size);

  std::pair<std::vector<uint8_t>, uint8_t> computeCommand(uint8_t cmd_type);
  void setRpmCallback(const hbot_msg::msg::Rpm::SharedPtr msg);

  rclcpp::Publisher<hbot_msg::msg::Rpm>::SharedPtr fb_rpm_pub_;
  rclcpp::Subscription<hbot_msg::msg::Rpm>::SharedPtr rpm_set_sub_;

  int baudrate_;
  std::string port_;
  serial::Serial* serial_;
  rclcpp::TimerBase::SharedPtr timer_receive_uart_;
  uint32_t rec_count_ = 0;
  uint32_t tran_count_ = 0;

  int16_t fb_rpm_left_, fb_rpm_right_;
  int16_t set_rpm_left_, set_rpm_right_;
  int16_t target_rpm_left_, target_rpm_right_;
  float kp_, ki_, kd_;

public:
  HbotDriver();
  ~HbotDriver();
};
