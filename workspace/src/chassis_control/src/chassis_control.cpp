#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <string>

// all nodes in ros2 are class
// so first step is to inherit from base class rclcpp::Node
class chassis_control : public rclcpp::Node {
public:
  // constructor
  chassis_control() : Node("chassis_control_node") {
    // set port and baudrate
    // values are pre-set in the program
    this->declare_parameter<std::string>("port", "/dev/arduino");
    this->declare_parameter<int>("baudrate", 115200);
    // assign value to private member
    port_ = this->get_parameter("port").get_parameter_value().string_value;
    baudrate_ =
        this->get_parameter("baudrate").get_parameter_value().integer_value;

    try {
      // init serial port
      serial_.setPort(port_);
      serial_.setBaudrate(baudrate_);
      serial_.setTimeout(serial::Timeout::simpleTimeout(1000));
      serial_.open();
      // check whether serial is open
      if (serial_.isOpen()) {
        RCLCPP_INFO(this->get_logger(), "Port is open: %s", port_.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open port");
      }
      timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&chassis_control::send_to_serial, this));
    } catch (const serial::IOException &e) {
      RCLCPP_ERROR(this->get_logger(), "Fail to open port: %s", e.what());
    }
  }

  ~chassis_control() {
    if (serial_.isOpen()) {
      serial_.close();
      RCLCPP_INFO(this->get_logger(), "Serial port closed.");
    }
  }

private:
  serial::Serial serial_;
  std::string port_;
  int baudrate_;
  rclcpp::TimerBase::SharedPtr timer_;

  void send_to_serial() {
    // message format: <linear velocity, angular velocity>
    std::string data = "<1,1>";

    if (serial_.isOpen()) {
      serial_.write(data); // send info
      RCLCPP_INFO(this->get_logger(), "Sent to Arduino: %s", data.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Serial port is not open!");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<chassis_control>());
  rclcpp::shutdown();
  return 0;
}