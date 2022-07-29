#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "hbot_driver/hbot_driver_node.h"

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

// #include "serial/serial.h"
// #include "hbot_msg/msg/rpm.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


#define debuglog true

HbotDriver::HbotDriver()
: Node("hbot_driver")
{
  // RCLCPP_INFO(this->get_logger(), "Get parameters");

  getParams();

  if (connectStm32()) {
    RCLCPP_INFO(this->get_logger(), "Connect to Stm32 successful");
  }

  // Read uart data every 20ms (50hz)
  timer_receive_uart_ = this->create_wall_timer(20ms, std::bind(&HbotDriver::readUart, this));
  on_parameter_changed_ = this->add_on_set_parameters_callback(
    std::bind(&HbotDriver::parametersCallback, this, _1));

  fb_rpm_pub_ = this->create_publisher<hbot_msg::msg::Rpm>("rpm_fb", 10);
  rpm_set_sub_ = this->create_subscription<hbot_msg::msg::Rpm>("rpm_in", 10, std::bind(&HbotDriver::setRpmCallback, this, _1));
  // while (1)
  // {
  //   std::string result = serial_->read(8U);

  // }

}

HbotDriver::~HbotDriver(){}

void HbotDriver::getParams() {
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  this->declare_parameter<float>("kp", 0.0);
  this->declare_parameter<float>("ki", 0.0);
  this->declare_parameter<float>("kd", 0.0);
  this->get_parameter<int>("baud_rate", baudrate_);
  this->get_parameter<std::string>("port", port_);
  this->get_parameter<float>("kp", kp_);
  this->get_parameter<float>("ki", ki_);
  this->get_parameter<float>("kd", kd_);
}

rcl_interfaces::msg::SetParametersResult HbotDriver::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &param: parameters) {
    RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
    RCLCPP_INFO(this->get_logger(), "%X", param.get_type());
    if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE){
      if(param.get_name() == "kp") {
        kp_ = param.get_value<float>();
        sendCommand(CMD_SET_KP);
      }
      if(param.get_name() == "ki") {
        ki_ = param.get_value<float>();
        RCLCPP_INFO(this->get_logger(), "set ki: %f", ki_);
        sendCommand(CMD_SET_KI);
      }
      if(param.get_name() == "kd") {
        kd_ = param.get_value<float>();
        RCLCPP_INFO(this->get_logger(), "set kd: %f", kd_);
        sendCommand(CMD_SET_KD);
      }
    }
  }
  return result;
}

bool HbotDriver::connectStm32() {
  try
  {
    serial_ = new serial::Serial(port_, baudrate_, serial::Timeout::simpleTimeout(1000));
    return serial_->isOpen() ? true : false;
  }
  catch(const serial::IOException ioe){
    RCLCPP_ERROR(get_logger(), "Could not open connection at " + port_);
    return 0;
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
}

void HbotDriver::readUart() {
  if(serial_->available() == 0) return;
  uint8_t start_msg;
  int8_t cnt_ = 0;
  bool isStartMsg = false;

  serial_->read(&start_msg, 1);
  isStartMsg = (start_msg == 0x2B);

  while (!isStartMsg || cnt_ > REC_UART_BUFFER_SIZE)
  {
    #if debuglog
    printf( "%02X cnt: %u\n", start_msg, cnt_);
    #endif
    serial_->read(&start_msg, 1);
    isStartMsg = (start_msg == 0x2B);
    cnt_++;
  }
  if(!isStartMsg) {
    RCLCPP_ERROR(get_logger(), "Could not detect start byte of message!");
    return;
  }
  uint8_t header[6];
  serial_->read(header, sizeof(header));

#if debuglog
  printf("%02X ", start_msg);
  for (size_t i = 0; i < sizeof(header); i++)
  {
    printf("%02X ",header[i]);
  }
  printf("\n");
  // // RCLCPP_INFO(get_logger(), ss.str());
#endif
  uint32_t new_cnt = (uint32_t)header[0] << 24
                      | (uint32_t)header[1] << 16
                      | (uint32_t)header[2] << 8
                      | header[3];
  if (new_cnt == rec_count_) return;
  rec_count_ = new_cnt;
  uint8_t msg_type = header[4];
  uint8_t msg_length = header[5];
  // printf("%u\n", msg_length);
  // printf("%u\n", header[0]);
  printf("Header: msg_length: %i, msg_type: %02X\n", msg_length, msg_type);

  // uint8_t parameters[msg_length-8];
  std::vector<uint8_t> msg;
  msg.resize(msg_length-7);
  try
  {
    serial_->read(msg, msg_length-7);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }

  float kpid;
  // printf("msg_type: %02X",msg_type);
  switch (msg_type)
  {
    case 0x00: // Feedback velocity
      fb_rpm_left_ = (int)msg.at(0) << 8 | msg.at(1);
      fb_rpm_right_ = (int)msg.at(2) << 8 | msg.at(3);
      RCLCPP_INFO(get_logger(), "Rpm feedback left: %d - right: %d", fb_rpm_left_, fb_rpm_right_);
      break;

    case 0x50: // Response received velocity command
      set_rpm_left_ = (int)msg.at(0) << 8 | msg.at(1);
      set_rpm_right_ = (int)msg.at(2) << 8 | msg.at(3);
      RCLCPP_INFO(get_logger(), "Stm32 received velocity: VL: %d - VL: %d", set_rpm_left_, set_rpm_right_);
      break;

    case 0x55: // set kp feedback
      kpid = static_cast<float>((int64_t)msg.at(0) << 24
                                | (int64_t)msg.at(1) << 16
                                | (int64_t)msg.at(2) << 8
                                | msg.at(3));
      RCLCPP_INFO(get_logger(), "Set Kp: %f", kpid);
      break;
    case 0x56: // set kp feedback
      kpid = static_cast<float>((int64_t)msg.at(0) << 24
                                | (int64_t)msg.at(1) << 16
                                | (int64_t)msg.at(2) << 8
                                | msg.at(3));
      RCLCPP_INFO(get_logger(), "Set Ki: %f", kpid);
      break;
    case 0x57: // set kp feedback
      kpid = static_cast<float>((int64_t)msg.at(0) << 24
                                | (int64_t)msg.at(1) << 16
                                | (int64_t)msg.at(2) << 8
                                | msg.at(3));
      RCLCPP_INFO(get_logger(), "Set Kd: %f", kpid);
      break;

    default:
      break;
  }
}

bool HbotDriver::sendCommand(uint8_t cmd_type) {
  std::vector<uint8_t> cmd;
  uint8_t cmd_size;
  tran_count_++;
  cmd.push_back(KEY_BEGIN_TRAN);
  cmd.push_back(tran_count_ >> 24);
  cmd.push_back(tran_count_ >> 16);
  cmd.push_back(tran_count_ >> 8);
  cmd.push_back(tran_count_);
  cmd.push_back(cmd_type);
  int k_int;
  switch (cmd_type)
  {
  case CMD_SET_VELOCITY:
    cmd_size = (uint8_t)12;
    cmd.push_back(cmd_size);
    cmd.push_back(target_rpm_left_ >> 8);
    cmd.push_back(target_rpm_left_);
    cmd.push_back(target_rpm_right_ >> 8);
    cmd.push_back(target_rpm_right_);
    break;
  case CMD_SET_KP:
    cmd_size = (uint8_t)12;
    cmd.push_back(cmd_size);
    k_int = static_cast<int>(kp_);
    cmd.push_back(k_int >> 24);
    cmd.push_back(k_int >> 16);
    cmd.push_back(k_int >> 8);
    cmd.push_back(k_int);
  case CMD_SET_KI:
    cmd_size = (uint8_t)12;
    cmd.push_back(cmd_size);
    k_int = static_cast<int>(ki_);
    cmd.push_back(k_int >> 24);
    cmd.push_back(k_int >> 16);
    cmd.push_back(k_int >> 8);
    cmd.push_back(k_int);
  case CMD_SET_KD:
    cmd_size = (uint8_t)12;
    cmd.push_back(cmd_size);
    k_int = static_cast<int>(ki_);
    cmd.push_back(k_int >> 24);
    cmd.push_back(k_int >> 16);
    cmd.push_back(k_int >> 8);
    cmd.push_back(k_int);
  default:
    break;
  }
  cmd.push_back(KEY_END_TRAN);

  if(serial_->write(cmd) == cmd_size) {return 1;}
  else {return 0;}
}

void HbotDriver::setRpmCallback(const hbot_msg::msg::Rpm::SharedPtr msg) {
  target_rpm_left_ = msg->left;
  target_rpm_right_ = msg->right;
  RCLCPP_INFO(get_logger(), "Set Rpm: Left: %d - Right: %d", target_rpm_left_, target_rpm_right_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HbotDriver>());
  rclcpp::shutdown();
  return 0;
}