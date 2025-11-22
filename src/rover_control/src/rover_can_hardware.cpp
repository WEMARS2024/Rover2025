#include "rover_control/rover_can_hardware.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <exception>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rover_control
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{
const char * joint_param_key(const std::string & joint_name)
{
  if (joint_name == "front_left_wheel_joint")
  {
    return "front_left_can_id";
  }
  if (joint_name == "front_right_wheel_joint")
  {
    return "front_right_can_id";
  }
  if (joint_name == "rear_left_wheel_joint")
  {
    return "rear_left_can_id";
  }
  if (joint_name == "rear_right_wheel_joint")
  {
    return "rear_right_can_id";
  }
  return nullptr;
}
}  

CallbackReturn RoverCanHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != joints_.size())
  {
    RCLCPP_FATAL(
      logger_, "Expected %zu wheel joints but got %zu", joints_.size(), info_.joints.size());
    return CallbackReturn::ERROR;
  }

  for (size_t index = 0; index < joints_.size(); ++index)
  {
    const auto & joint_info = info_.joints[index];
    joints_[index].name = joint_info.name;

    if (joint_info.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        logger_, "Joint '%s' has %zu command interfaces. 1 expected.",
        joint_info.name.c_str(), joint_info.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint_info.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        logger_, "Joint '%s' command interface '%s'. '%s' expected.",
        joint_info.name.c_str(), joint_info.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint_info.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        logger_, "Joint '%s' has %zu state interfaces. 2 expected.",
        joint_info.name.c_str(), joint_info.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint_info.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint_info.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        logger_, "Joint '%s' state interfaces must be position and velocity.",
        joint_info.name.c_str());
      return CallbackReturn::ERROR;
    }

    const char * param_key = joint_param_key(joint_info.name);
    if (param_key == nullptr)
    {
      RCLCPP_FATAL(logger_, "Joint '%s' is not supported by RoverCanHardware", joint_info.name.c_str());
      return CallbackReturn::ERROR;
    }

    const auto param_it = info_.hardware_parameters.find(param_key);
    if (param_it == info_.hardware_parameters.end())
    {
      RCLCPP_FATAL(
        logger_, "Missing hardware parameter '%s' for joint '%s'", param_key, joint_info.name.c_str());
      return CallbackReturn::ERROR;
    }

    try
    {
      joints_[index].can_id = std::stoi(param_it->second);
    }
    catch (const std::exception & e)
    {
      RCLCPP_FATAL(
        logger_, "Failed parsing '%s' for joint '%s': %s", param_key, joint_info.name.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
  }

  const auto can_param = info_.hardware_parameters.find("can_interface");
  if (can_param != info_.hardware_parameters.end())
  {
    can_interface_name_ = can_param->second;
  }

  RCLCPP_INFO(
    logger_, "Configured CAN interface '%s' with %zu wheel joints",
    can_interface_name_.c_str(), joints_.size());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoverCanHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joints_.size() * 2);
  for (auto & joint : joints_)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION, &joint.pos));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.vel));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoverCanHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joints_.size());
  for (auto & joint : joints_)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &joint.cmd_vel));
  }
  return command_interfaces;
}

CallbackReturn RoverCanHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating Rover CAN");
  can_connected_ = connect_can_interface();
  if (!can_connected_)
  {
    RCLCPP_FATAL(
      logger_, "Failed to connect to CAN interface '%s'", can_interface_name_.c_str());
    return CallbackReturn::ERROR;
  }

  for (auto & joint : joints_)
  {
    joint.cmd_vel = 0.0;
    joint.vel = 0.0;
  }

  RCLCPP_INFO(logger_, "Rover CAN activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoverCanHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating Rover CAN");
  for (auto & joint : joints_)
  {
    joint.cmd_vel = 0.0;
  }
  if (can_socket_ >= 0)
  {
    ::close(can_socket_);
    can_socket_ = -1;
  }
  can_connected_ = false;
  RCLCPP_INFO(logger_, "Rover CAN hardware deactivated");
  return CallbackReturn::SUCCESS;
}

return_type RoverCanHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  for (auto & joint : joints_)
  {
    if (can_connected_)
    {
      if (!read_joint_state(joint))
      {
        RCLCPP_WARN(
          logger_, "Failed reading joint state for '%s' (id %d)", joint.name.c_str(), joint.can_id);
      }
    }

    joint.pos += joint.vel * dt;
  }

  return return_type::OK;
}

return_type RoverCanHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & joint : joints_)
  {
    joint.vel = joint.cmd_vel;

    if (can_connected_ && !send_velocity_command(joint))
    {
      RCLCPP_WARN(
        logger_, "Failed sending velocity command for '%s' (id %d)", joint.name.c_str(), joint.can_id);
    }
  }

  return return_type::OK;
}

bool RoverCanHardware::connect_can_interface()
{
  if (can_socket_ >= 0)
  {
    ::close(can_socket_);
    can_socket_ = -1;
  }

  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0)
  {
    RCLCPP_ERROR(
      logger_, "Failed to open CAN socket on '%s': %s",
      can_interface_name_.c_str(), std::strerror(errno));
    return false;
  }

  struct ifreq ifr
  {
  };
  std::strncpy(ifr.ifr_name, can_interface_name_.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
  {
    RCLCPP_ERROR(
      logger_, "Failed to resolve interface '%s': %s",
      can_interface_name_.c_str(), std::strerror(errno));
    ::close(fd);
    return false;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(
      logger_, "Failed to bind CAN interface '%s': %s",
      can_interface_name_.c_str(), std::strerror(errno));
    ::close(fd);
    return false;
  }

  can_socket_ = fd;
  RCLCPP_INFO(logger_, "Connected to SocketCAN interface '%s'", can_interface_name_.c_str());
  return true;
}

bool RoverCanHardware::send_velocity_command(const JointData & joint)
{
  if (can_socket_ < 0)
  {
    return false;
  }

  const uint32_t arbitration_id = 0x02050080U + static_cast<uint32_t>(joint.can_id);
  const float velocity = static_cast<float>(joint.cmd_vel);

  struct can_frame frame
  {
  };
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = static_cast<canid_t>(arbitration_id);
  frame.can_dlc = 8;
  std::memcpy(frame.data, &velocity, sizeof(float));

  const ssize_t bytes = ::write(can_socket_, &frame, sizeof(frame));
  if (bytes != static_cast<ssize_t>(sizeof(frame)))
  {
    RCLCPP_ERROR(
      logger_, "SocketCAN write failed for '%s': %s",
      joint.name.c_str(), std::strerror(errno));
    return false;
  }

  char frame_string[8 + 1 + 16 + 1]{0};
  std::snprintf(
    frame_string, sizeof(frame_string), "%08X#%02X%02X%02X%02X00000000",
    arbitration_id, frame.data[0], frame.data[1], frame.data[2], frame.data[3]);
  RCLCPP_DEBUG(logger_, "CAN frame %s", frame_string);
  return true;
}

}  


PLUGINLIB_EXPORT_CLASS(
  rover_control::RoverCanHardware,
  hardware_interface::SystemInterface)
