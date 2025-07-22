#include "cobot_hardware/cobot_hardware.hpp"

#include <sys/ioctl.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "libserial/SerialPort.h"

namespace cobot_hardware
{

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
  joints.resize(info_.joints.size(), Joint());
  for (size_t i=0; i<info_.joints.size(); i++) {
    joints.at(i).name = info_.joints.at(i).name;
    joints.at(i).id = static_cast<uint8_t>(std::stoi(info_.joints.at(i).parameters.at("id")));
    joints.at(i).state = std::numeric_limits<double>::quiet_NaN();
    joints.at(i).command = std::numeric_limits<double>::quiet_NaN();
    joints.at(i).prevCommand = joints.at(i).command;
    RCLCPP_INFO(logger, "Joint ID : %d", joints.at(i).id);
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

  // TODO: read write sekali
  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

// ==================================================
//                        read              
// ==================================================

return_type CobotHardware::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
  // TODO: read
  char startMarker;
  do {
    try {
      port.ReadByte(startMarker, ms_timeout);
    }
    catch(const LibSerial::ReadTimeout&) {
      RCLCPP_ERROR(logger, "read '<' timeout.");
      return return_type::ERROR;
    }
  } while (startMarker != '<');
  
  LibSerial::DataBuffer buffer(intSize);
  for (auto& joint : joints) {
    try {
      port.Read(buffer, intSize, ms_timeout);
    }
    catch(const LibSerial::ReadTimeout&) {
      RCLCPP_ERROR(logger, "read data timeout");
      return return_type::ERROR;
    }

    int intVal;
    std::memcpy(&intVal, buffer.data(), buffer.size());
    joint.state = static_cast<double>(intVal);
  }
  
  return return_type::OK;
}

// ==================================================
//                       write              
// ==================================================

return_type CobotHardware::write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
  const int jointCount = joints.size();
  const int dataSize = jointCount * intSize;
  const int totalSize = dataSize + 2;  // '<' + data + '>'

  LibSerial::DataBuffer packet(totalSize);
  packet[0] = '<';

  int indexPos = 1;
  for (const auto& joint: joints) {
    int pos = static_cast<int>(joint.command);
    std::memcpy(&packet[indexPos], &pos, intSize);
    indexPos += intSize;
  }

  packet[indexPos] = '>';

  port.Write(packet);

  return return_type::OK;
}

}  // namespace cobot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cobot_hardware::CobotHardware, hardware_interface::SystemInterface)