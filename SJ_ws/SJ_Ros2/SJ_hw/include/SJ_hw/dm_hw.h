/*** 
 * @Author: _yerik
 * @Date: 2025-07-22 14:21:17
 * @LastEditTime: 2025-07-23 00:38:40
 * @LastEditors: _yerik
 * @Description: 
 * @FilePath: /Simple_Joint/SJ_ws/SJ_Ros2/SJ_hw/include/SJ_hw/dm_hw.h
 * @Code. Run. No errors.
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "SJ_hw/damiao.h" 

namespace damiao
{

class DmHW : public hardware_interface::SystemInterface
{
public:
  
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  //Get data from urdf and Store real-time data for all joints
  std::vector<damiao::DmActData> hw_actuator_data_;

  //Serial port to motor mapping relationship
  std::unordered_map<std::string, std::unique_ptr<damiao::Motor_Control>> motor_controls_;

  //Joint-to-motor mapping relationships
  std::unordered_map<int, damiao::Motor_Control*> joint_to_driver_map_;

  std::unordered_map<std::string, std::unordered_map<int, DmActData>> port_to_motors_config_;
};

RCLCPP_SHARED_PTR_DEFINITIONS(DmHW)

} // namespace damiao