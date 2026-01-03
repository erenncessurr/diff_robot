#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp" 
#include "rclcpp/macros.hpp" 
#include <libserial/SerialPort.h> 
#include <vector>
#include <string>

namespace diff_robot
{
class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  LibSerial::SerialPort serial_device_; // Seri port nesnesi
  std::string port_name_;

  // Komutlar (Controller'dan gelen)
  double left_wheel_cmd_ = 0.0;
  double right_wheel_cmd_ = 0.0;

  // Durumlar (Sisteme geri beslenen)
  double left_wheel_pos_ = 0.0;
  double left_wheel_vel_ = 0.0;
  double right_wheel_pos_ = 0.0;
  double right_wheel_vel_ = 0.0;
};
}