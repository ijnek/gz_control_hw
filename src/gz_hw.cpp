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

#include <vector>

#include "gz_control_hw/gz_hw.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ignition/transport/Node.hh"
#include "rclcpp/rclcpp.hpp"

namespace gz_control_hw
{

class Joint
{
public:
  std::string name;
  double position_state;
  double position_command;
  ignition::transport::Node::Publisher publisher;
};

class GzHwPrivate
{
public:
  GzHwPrivate() = default;
  ~GzHwPrivate() = default;

  std::vector<Joint> joints;
  ignition::transport::Node node;
};

hardware_interface::CallbackReturn GzHw::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  this->dataPtr = std::make_unique<GzHwPrivate>();

  for (const auto & joint : info.joints) {
    Joint j;
    j.name = joint.name;
    j.position_state = std::numeric_limits<double>::quiet_NaN();
    j.position_command = std::numeric_limits<double>::quiet_NaN();

    try {
      std::string robot_name = info.hardware_parameters.at("robot_name");
      j.publisher = this->dataPtr->node.Advertise<ignition::msgs::Double>(
        "/model/" + robot_name + "/joint/" + joint.name + "/0/cmd_pos");
    } catch(std::out_of_range&) {
      RCLCPP_ERROR(rclcpp::get_logger("gz_hw"), "<param name=\"robot_name\">my_robot</param> not found under <hardware> under <ros2_control>");
      return hardware_interface::CallbackReturn::ERROR;
    }

    this->dataPtr->joints.push_back(j);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GzHw::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GzHw::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & joint : this->dataPtr->joints) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.position_state));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GzHw::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : this->dataPtr->joints) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.position_command));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn GzHw::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GzHw::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GzHw::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GzHw::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (auto & joint : this->dataPtr->joints) {
    ignition::msgs::Double msg;
    msg.set_data(joint.position_command);
    joint.publisher.Publish(msg);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace gz_control_hw


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gz_control_hw::GzHw, hardware_interface::SystemInterface)
