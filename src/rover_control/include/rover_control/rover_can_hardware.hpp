#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rover_control
{

class RoverCanHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoverCanHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct JointData
  {
    std::string name;
    double cmd_vel{0.0};
    double pos{0.0};
    double vel{0.0};
    int can_id{0};
  };

  bool connect_can_interface();
  bool send_velocity_command(const JointData & joint);
  bool read_joint_state(JointData & joint);
  std::array<JointData, 4> joints_;
  std::string can_interface_name_{"can0"};
  int can_socket_{-1};
  bool can_connected_{false};
  rclcpp::Logger logger_{rclcpp::get_logger("RoverCanHardware")};
};

}  
