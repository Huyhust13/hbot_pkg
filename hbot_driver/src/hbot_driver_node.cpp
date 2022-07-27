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
  RCLCPP_INFO(this->get_logger(), "Get parameters");
  getParams();

  if (!connectStm32()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not connect to stm32 with port: " << port_ << " and baud rate: " << baudrate_);
    // return;
  }
  else {
    RCLCPP_INFO(this->get_logger(), "Connect to Stm32 successful");
  }

  // Read uart data every 20ms (50hz)
  timer_receive_uart_ = this->create_wall_timer(20ms, std::bind(&HbotDriver::readUart, this));

  fb_rpm_pub_ = this->create_publisher<hbot_msg::msg::Rpm>("rpm_fb", 10);
  rpm_set_sub_ = this->create_subscription<hbot_msg::msg::Rpm>("rpm_in", 10, std::bind(&HbotDriver::setRpmCallback, this, _1));
  // while (1)
  // {
  //   std::string result = serial_->read(8U);

  // }

}

HbotDriver::~HbotDriver(){}

void HbotDriver::getParams() {
  this->get_parameter_or<int>("baud_rate", baudrate_, 115200);
  this->get_parameter_or<std::string>("port", port_, "/dev/ttyUSB0");
}

bool HbotDriver::connectStm32() {
  serial_ = new serial::Serial(port_, baudrate_, serial::Timeout::simpleTimeout(1000));
  return serial_->isOpen() ? true : false;
}

void HbotDriver::readUart() {
  auto begin = rclcpp::Clock().now().nanoseconds();
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
  uint8_t msg_length = header[0];
  // printf("%u\n", msg_length);
  // printf("%u\n", header[0]);
  uint8_t msg_type = header[1];
  uint32_t curr_count = (uint32_t)header[2] << 24 | (uint32_t)header[3] << 16 | (uint32_t)header[4] << 8 | header[5];

  printf("Header: msg_length: %i, msg_type: %02X\n", msg_length, msg_type);
  if (curr_count == rec_count_) return;
  rec_count_ = curr_count;
  uint8_t parameters[msg_length-8];
  for(int i=0;i<4;i++){
  try
  {
    /* code */
    // uint8_t tmp;
    // serial_->read(&tmp, 1);
    // printf("%02X ");
    serial_->read(parameters, sizeof(parameters));
  }
  catch(const std::exception& e)
  {
    std::cerr << 'asdfasdf' << e.what() << '\n';
  }
  }
  printf("\n");

  // printf("msg_type: %02X",msg_type);
  switch (msg_type)
  {
    case 0x00: // Feedback velocity
      fb_rpm_left_ = (int)parameters[0] << 8 | parameters[1];
      fb_rpm_right_ = (int)parameters[2] << 8 | parameters[3];
      RCLCPP_INFO(get_logger(), "Rpm feedback left: %d - right: %d", fb_rpm_left_, fb_rpm_right_);
      break;

    case 0x10: // Response received velocity command
      set_rpm_left_ = (int)parameters[0] << 8 | parameters[1];
      set_rpm_right_ = (int)parameters[2] << 8 | parameters[3];
      RCLCPP_INFO(get_logger(), "Stm32 received velocity: VL: %d - VL: %d", set_rpm_left_, set_rpm_right_);
      break;

    default:
      break;
  }
  uint8_t end_msg;
  serial_->read(&end_msg, 1);
  std::cout << "Period: " << (rclcpp::Clock().now().nanoseconds() - begin)/1E6 << "(ms)"<< std::endl;
}

bool HbotDriver::sendCommand(std::vector<uint8_t> buff, uint8_t size){
  if(serial_->write(buff) != size) return 0;
  return 1;
}

std::pair<std::vector<uint8_t>, uint8_t> HbotDriver::computeCommand(uint8_t cmd_type) {
  std::pair<std::vector<uint8_t>, uint8_t> cmd;
  tran_count_++;
  cmd.first.push_back(KEY_BEGIN_TRAN);
  cmd.first.push_back(tran_count_ >> 24);
  cmd.first.push_back(tran_count_ >> 16);
  cmd.first.push_back(tran_count_ >> 8);
  cmd.first.push_back(tran_count_);
  cmd.first.push_back(cmd_type);
  int k_int;
  switch (cmd_type)
  {
  case CMD_SET_VELOCITY:
    cmd.second = (uint8_t)12;
    cmd.first.push_back(cmd.second);
    cmd.first.push_back(target_rpm_left_ >> 8);
    cmd.first.push_back(target_rpm_left_);
    cmd.first.push_back(target_rpm_right_ >> 8);
    cmd.first.push_back(target_rpm_right_);
    break;
  case CMD_SET_KP:
    cmd.second = (uint8_t)12;
    cmd.first.push_back(cmd.second);
    k_int = static_cast<int>(kp_);
    cmd.first.push_back(k_int >> 24);
    cmd.first.push_back(k_int >> 16);
    cmd.first.push_back(k_int >> 8);
    cmd.first.push_back(k_int);
  case CMD_SET_KI:
    cmd.second = (uint8_t)12;
    cmd.first.push_back(cmd.second);
    k_int = static_cast<int>(ki_);
    cmd.first.push_back(k_int >> 24);
    cmd.first.push_back(k_int >> 16);
    cmd.first.push_back(k_int >> 8);
    cmd.first.push_back(k_int);
  case CMD_SET_KD:
    cmd.second = (uint8_t)12;
    cmd.first.push_back(cmd.second);
    k_int = static_cast<int>(ki_);
    cmd.first.push_back(k_int >> 24);
    cmd.first.push_back(k_int >> 16);
    cmd.first.push_back(k_int >> 8);
    cmd.first.push_back(k_int);
  default:
    break;
  }
  cmd.first.push_back(KEY_END_TRAN);
  return cmd;
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