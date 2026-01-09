#include "diff_robot/diff_robot.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diff_robot
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Parametreleri çek
  port_name_ = info.hardware_parameters.at("device");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    serial_device_.Open(port_name_);
    serial_device_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    RCLCPP_INFO(rclcpp::get_logger("DiffBotHardware"), "Seri port acildi: %s", port_name_.c_str());
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotHardware"), "Seri port ACILAMADI: %s", port_name_.c_str());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  serial_device_.Close();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  // ENCODER OLMADIGI ICIN: Komut edilen hızı gerçek hız gibi kabul et (Open Loop)
  left_wheel_vel_ = left_wheel_cmd_;
  right_wheel_vel_ = right_wheel_cmd_;

  // Pozisyonu hesapla (Hız * Zaman)
  left_wheel_pos_ += left_wheel_vel_ * period.seconds();
  right_wheel_pos_ += right_wheel_vel_ * period.seconds();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // ESP32'ye gönderilecek mesajı hazırla: "v [sol] [sag]\n"
  std::string msg = "v " + std::to_string(left_wheel_cmd_) + " " + std::to_string(right_wheel_cmd_) + "\n";
  
  try {
    serial_device_.Write(msg);
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotHardware"), "Seri porta yazilamadi!");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_POSITION, &left_wheel_pos_);
  state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &left_wheel_vel_);
  state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_POSITION, &right_wheel_pos_);
  state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &right_wheel_vel_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &left_wheel_cmd_);
  command_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &right_wheel_cmd_);
  return command_interfaces;
}
} // namespace diff_robot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diff_robot::DiffBotSystemHardware, hardware_interface::SystemInterface)