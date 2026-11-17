#ifndef DIFFDRIVE_ARDUINO_FAKE_ROBOT_H
#define DIFFDRIVE_ARDUINO_FAKE_ROBOT_H

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "config.h"
#include "wheel.h"

using hardware_interface::return_type;

class FakeRobot : public hardware_interface::SystemInterface
{
public:
  FakeRobot();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  Config cfg_;

  Wheel l_wheel_;
  Wheel r_wheel_;

  rclcpp::Logger logger_;
};

#endif
