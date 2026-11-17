#include "diffdrive_arduino/fake_robot.h"

FakeRobot::FakeRobot()
: logger_(rclcpp::get_logger("FakeRobot"))
{
}

hardware_interface::CallbackReturn FakeRobot::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring fake differential drive robot");

  cfg_.left_wheel_name = info_.hardware_parameters.at("left_wheel_name");
  cfg_.right_wheel_name = info_.hardware_parameters.at("right_wheel_name");

  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  RCLCPP_INFO(logger_, "Fake robot configured");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
FakeRobot::export_state_interfaces()
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
FakeRobot::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd);

  command_interfaces.emplace_back(
    r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd);

  return command_interfaces;
}

hardware_interface::CallbackReturn FakeRobot::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating fake robot");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeRobot::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating fake robot");

  l_wheel_.cmd = 0.0;
  r_wheel_.cmd = 0.0;
  l_wheel_.vel = 0.0;
  r_wheel_.vel = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

return_type FakeRobot::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  l_wheel_.pos += l_wheel_.vel * dt;
  r_wheel_.pos += r_wheel_.vel * dt;

  return return_type::OK;
}

return_type FakeRobot::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  l_wheel_.vel = l_wheel_.cmd;
  r_wheel_.vel = r_wheel_.cmd;

  return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  FakeRobot,
  hardware_interface::SystemInterface
)
