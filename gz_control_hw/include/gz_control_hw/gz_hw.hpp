// Copyright 2022 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GZ_CONTROL_HW__GZ_HW_HPP_
#define GZ_CONTROL_HW__GZ_HW_HPP_

#include "gz_control_hw/visibility_control.hpp"
#include "hardware_interface/system_interface.hpp"

namespace gz_control_hw
{

class GzHw : public hardware_interface::SystemInterface
{
public:
  GZ_CONTROL_HW_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  GZ_CONTROL_HW_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  GZ_CONTROL_HW_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  GZ_CONTROL_HW_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  GZ_CONTROL_HW_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  GZ_CONTROL_HW_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  GZ_CONTROL_HW_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  GZ_CONTROL_HW_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
};

}  // namespace gz_control_hw

#endif  // GZ_CONTROL_HW__GZ_HW_HPP_
