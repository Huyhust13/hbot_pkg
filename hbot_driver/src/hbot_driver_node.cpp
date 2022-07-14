#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "serial/serial.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

constexpr int8_t REC_UART_BUFFER_SIZE = 12;

#define debuglog true

class HbotDriver : public rclcpp::Node
{
  public:
    HbotDriver()
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

      timer_receive_uart_ = this->create_wall_timer(20ms, std::bind(&HbotDriver::readUart, this));

      // while (1)
      // {
      //   std::string result = serial_->read(8U);

      // }


    }

  private:
    void getParams() {
      this->get_parameter_or<int>("baud_rate", baudrate_, 115200);
      this->get_parameter_or<std::string>("port", port_, "/dev/ttyUSB0");
    }

    void readUart() {
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
        printf( "%02X cnt: %i\n", start_msg, cnt_);
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
      // printf("%i\n", msg_length);
      // printf("%i\n", header[0]);
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
          RCLCPP_INFO(get_logger(), "rpm feedback left: %i - right: %i", fb_rpm_left_, fb_rpm_right_);
          break;

        case 0x10: // Response received velocity command
          set_rpm_left_ = (int)parameters[0] << 8 | parameters[1];
          set_rpm_right_ = (int)parameters[2] << 8 | parameters[3];
          RCLCPP_INFO(get_logger(), "Received velocity: VL: %i - VL: %i", set_rpm_left_, set_rpm_right_);
          break;

        default:
          break;
      }
      uint8_t end_msg;
      serial_->read(&end_msg, 1);
      std::cout << "Period: " << (rclcpp::Clock().now().nanoseconds() - begin)/1E6 << "(ms)"<< std::endl;
    }

    bool connectStm32() {
      serial_ = new serial::Serial(port_, baudrate_, serial::Timeout::simpleTimeout(1000));
      return serial_->isOpen() ? true : false;
    }

    int baudrate_;
    std::string port_;
    serial::Serial* serial_;
    rclcpp::TimerBase::SharedPtr timer_receive_uart_;
    uint8_t rec_buffer_[REC_UART_BUFFER_SIZE];
    uint32_t rec_count_ = 0;
    int16_t fb_rpm_left_, fb_rpm_right_;
    int16_t set_rpm_left_, set_rpm_right_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HbotDriver>());
  rclcpp::shutdown();
  return 0;
}