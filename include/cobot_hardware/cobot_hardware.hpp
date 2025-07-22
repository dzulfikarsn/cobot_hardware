#ifndef COBOT_HARDWARE__COBOT_HARDWARE_HPP_
#define COBOT_HARDWARE__COBOT_HARDWARE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"

#include "libserial/SerialPort.h"

namespace cobot_hardware
{

constexpr int intSize = 4;
constexpr int ms_timeout = 100;

struct Joint
{
  uint8_t id{};
  std::string name;
  double state{};
  double command{};
  double prevCommand{};
};

class CobotHardware : public hardware_interface::SystemInterface
{
public:
  CobotHardware();
  ~CobotHardware() = default;

  RCLCPP_SHARED_PTR_DEFINITIONS(CobotHardware)

  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  
  /**
   * @brief menuliskan data sudut servo ke serial
   * 
   * @note format : '<' + data + '>'
   *       dimana data : nilai integer sejumlah servo yang dibuat dalam vector of uint8_t
   */
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

private:
  hardware_interface::return_type enable_torque(const bool enabled);

  hardware_interface::return_type reset_command();

  hardware_interface::return_type set_position_mode();

  std::vector<Joint> joints;
  LibSerial::SerialPort port;
  std::string portName;
};

}

#endif  //COBOT_HARDWARE__COBOT_HARDWARE_HPP_