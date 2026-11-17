#include "diffdrive_arduino/diffdrive_arduino.h"

#include <string>
#include <exception>

DiffDriveArduino::DiffDriveArduino()
: logger_(rclcpp::get_logger("DiffDriveArduino"))
{
}

hardware_interface::CallbackReturn DiffDriveArduino::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring Arduino differential drive");

  try
  {
    cfg_.left_wheel_name = info_.hardware_parameters.at("left_wheel_name");
    cfg_.right_wheel_name = info_.hardware_parameters.at("right_wheel_name");

    cfg_.loop_rate = std::stof(info_.hardware_parameters.at("loop_rate"));
    cfg_.device = info_.hardware_parameters.at("device");
    cfg_.baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
    cfg_.timeout = std::stoi(info_.hardware_parameters.at("timeout"));
    cfg_.enc_counts_per_rev =
      std::stoi(info_.hardware_parameters.at("enc_counts_per_rev"));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(logger_, "Hardware parameter error: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

  RCLCPP_INFO(logger_, "Arduino hardware configured");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DiffDriveArduino::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel);

  state_interfaces.emplace_back(
    l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos);

  state_interfaces.emplace_back(
    r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel);

  state_interfaces.emplace_back(
    r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DiffDriveArduino::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd);

  command_interfaces.emplace_back(
    r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd);

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveArduino::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating Arduino controller");

  if (!arduino_.connected())
  {
    RCLCPP_ERROR(logger_, "Arduino is not connected");
    return hardware_interface::CallbackReturn::ERROR;
  }

  arduino_.sendEmptyMsg();
  arduino_.setPidValues(30, 20, 0, 100);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduino::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating Arduino controller");

  if (arduino_.connected())
  {
    arduino_.setMotorValues(0, 0);
  }

  l_wheel_.cmd = 0.0;
  r_wheel_.cmd = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

return_type DiffDriveArduino::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (!arduino_.connected())
  {
    RCLCPP_ERROR(logger_, "Arduino disconnected");
    return return_type::ERROR;
  }

  const double dt = period.seconds();

  if (dt <= 0.0)
  {
    return return_type::OK;
  }

  arduino_.readEncoderValues(l_wheel_.enc, r_wheel_.enc);

  double previous_position = l_wheel_.pos;

  l_wheel_.pos = l_wheel_.calcEncAngle();
  l_wheel_.vel = (l_wheel_.pos - previous_position) / dt;

  previous_position = r_wheel_.pos;

  r_wheel_.pos = r_wheel_.calcEncAngle();
  r_wheel_.vel = (r_wheel_.pos - previous_position) / dt;

  return return_type::OK;
}

return_type DiffDriveArduino::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  arduino_.setMotorValues(
    l_wheel_.cmd / l_wheel_.rads_per_count / cfg_.loop_rate,
    r_wheel_.cmd / r_wheel_.rads_per_count / cfg_.loop_rate
  );

  return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)
