#include "cobot_hardware/cobot_hardware.hpp"

#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "libserial/SerialPort.h"

namespace cobot_hardware
{

constexpr int INT_SIZE = 4;  // sizeof(int)
constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double DEG_TO_RAD = M_PI / 180.0;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

rclcpp::Logger logger = rclcpp::get_logger("CobotHardware");

// ==================================================
//                       on_init                     
// ==================================================

CallbackReturn CobotHardware::on_init(const hardware_interface::HardwareInfo& info) {
  RCLCPP_DEBUG(logger, "on_init");

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // ambil data dari ros2control tag
  joints.clear();
  for (const auto& infoJoint : info_.joints) {
    Joint j;
    j.name = infoJoint.name;
    j.id = static_cast<uint8_t>(std::stoi(infoJoint.parameters.at("id")));
    j.command = 0.0;
    joints.push_back(j);
    RCLCPP_INFO(logger, "Joint ID : %d", j.id);
  }

  portName = info_.hardware_parameters.at("port_name");

  return CallbackReturn::SUCCESS;
}

// ==================================================
//             export_command_interfaces             
// ==================================================

std::vector<hardware_interface::CommandInterface> CobotHardware::export_command_interfaces() {
  RCLCPP_DEBUG(logger, "export_command_interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto& joint : joints) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.command));
  }

  return command_interfaces;
}

// ==================================================
//              export_state_interfaces              
// ==================================================

std::vector<hardware_interface::StateInterface> CobotHardware::export_state_interfaces() {
  RCLCPP_DEBUG(logger, "export_state_interfaces");

  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto& joint : joints) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.state));
  }

  return state_interfaces;
}

// ==================================================
//                    on_configure                     
// ==================================================

CallbackReturn CobotHardware::on_configure(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_DEBUG(logger, "configure");

  // TODO: nyambung ke mikon
  try {
    port.Open(portName);  // throw if fail
    port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    port.SetParity(LibSerial::Parity::PARITY_NONE);
    port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  }
  catch (const LibSerial::OpenFailed&) {
    RCLCPP_FATAL(logger, "The serial port: %s did not open correctly.", portName.c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger, "The serial port: %s opened.", portName.c_str());

  return CallbackReturn::SUCCESS;
}

// ==================================================
//                     on_activate                     
// ==================================================

CallbackReturn CobotHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  RCLCPP_DEBUG(logger, "activate");

  // TODO: write sekali
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

// ==================================================
//                        read              
// ==================================================

return_type CobotHardware::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
  for (auto& joint : joints) {
    joint.state = joint.deg * DEG_TO_RAD;
  }
  
  return return_type::OK;
}

// ==================================================
//                       write              
// ==================================================

return_type CobotHardware::write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
  // convert rad to deg
  for (auto& joint : joints) {
    joint.deg = joint.command * RAD_TO_DEG;
  }

  // WRITE SESSION

  const int dataSize = 1 + joints.size() * INT_SIZE + 1;  // '<' + data + '>'

  LibSerial::DataBuffer packet(dataSize);  // LibSerial::DataBuffer = std::vector<uint8_t>
  packet[0] = '<';  // start marker

  int indexPos = 1;  // the index after '<'
  for (const auto& joint: joints) {
    int pos = static_cast<int>(std::round(joint.deg));
    std::memcpy(&packet[indexPos], &pos, INT_SIZE);
    indexPos += INT_SIZE;
  }

  packet[indexPos] = '>';  // end marker

  port.Write(packet);

  return return_type::OK;
}

}  // namespace cobot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cobot_hardware::CobotHardware, hardware_interface::SystemInterface)