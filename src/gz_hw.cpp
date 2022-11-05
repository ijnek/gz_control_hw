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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gz_control_hw/gz_hw.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ignition/transport/Node.hh"
#include "rclcpp/rclcpp.hpp"
#include "ros_ign_bridge/convert/sensor_msgs.hpp"

namespace gz_control_hw
{

struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

class Joint
{
public:
  std::string name;
  JointValue state;
  JointValue command;
  ignition::transport::Node::Publisher pub_cmd_pos;
  ignition::transport::Node::Publisher pub_cmd_vel;
};

class GzHwPrivate
{
public:
  GzHwPrivate() = default;
  ~GzHwPrivate() = default;
  void jointStateCallback(const ignition::msgs::Model & ignMsg)
  {
    sensor_msgs::msg::JointState jointState;
    ros_ign_bridge::convert_ign_to_ros(ignMsg, jointState);
    for (auto & joint : joints) {
      auto it = find(jointState.name.begin(), jointState.name.end(), joint.name);
      if (it != jointState.name.end()) {
        int i = it - jointState.name.begin();
        joint.state.position = jointState.position[i];
        joint.state.velocity = jointState.velocity[i];
        joint.state.effort = jointState.effort[i];
      }
    }
  }

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

  // Read robot_name hardware parameter
  std::string robot_name;
  try {
    robot_name = info.hardware_parameters.at("robot_name");
  } catch (std::out_of_range &) {
    RCLCPP_ERROR(
      rclcpp::get_logger("gz_hw"),
      "<param name=\"robot_name\">my_robot</param> not found under <hardware> under "
      "<ros2_control>");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read joint_states_ign_topic hardware parameter
  std::string joint_states_ign_topic;
  try {
    joint_states_ign_topic = info.hardware_parameters.at("joint_states_ign_topic");
  } catch (std::out_of_range &) {
    RCLCPP_ERROR(
      rclcpp::get_logger("gz_hw"),
      "<param name=\"joint_states_ign_topic\">my_topic</param> not found under <hardware> under "
      "<ros2_control>");
    return hardware_interface::CallbackReturn::ERROR;
  }

  this->dataPtr = std::make_unique<GzHwPrivate>();

  for (const auto & joint : info.joints) {
    Joint j;
    j.name = joint.name;
    j.state.position = std::numeric_limits<double>::quiet_NaN();
    j.command.position = std::numeric_limits<double>::quiet_NaN();
    j.pub_cmd_pos = this->dataPtr->node.Advertise<ignition::msgs::Double>(
      "/model/" + robot_name + "/joint/" + joint.name + "/0/cmd_pos");
    j.pub_cmd_vel = this->dataPtr->node.Advertise<ignition::msgs::Double>(
      "/model/" + robot_name + "/joint/" + joint.name + "/cmd_vel");
    this->dataPtr->joints.push_back(j);
  }

  this->dataPtr->node.Subscribe(
    joint_states_ign_topic, &GzHwPrivate::jointStateCallback, this->dataPtr.get());

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
        joint.name, hardware_interface::HW_IF_POSITION, &joint.state.position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &joint.state.velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_EFFORT, &joint.state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GzHw::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : this->dataPtr->joints) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.command.position));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &joint.command.velocity));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_EFFORT, &joint.command.effort));
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
  auto joints = this->dataPtr->joints;

  bool receivedVelocityCmd = std::any_of(
    joints.cbegin(), joints.cend(), [](auto j) {return j.command.velocity != 0.0;});
  bool receivedEffortCmd = std::any_of(
    joints.cbegin(), joints.cend(), [](auto j) {return j.command.effort != 0.0;});

  if (receivedVelocityCmd) {
    // Velocity control
    for (auto & joint : joints) {
      ignition::msgs::Double msg;
      msg.set_data(joint.command.velocity);
      joint.pub_cmd_vel.Publish(msg);
    }
  } else if (receivedEffortCmd) {
    // Effort control
    RCLCPP_ERROR(rclcpp::get_logger("gz_hw"), "Effort control is not implemented");
    return hardware_interface::return_type::ERROR;
  } else {
    // Position control
    for (auto & joint : joints) {
      ignition::msgs::Double msg;
      msg.set_data(joint.command.position);
      joint.pub_cmd_pos.Publish(msg);
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace gz_control_hw


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gz_control_hw::GzHw, hardware_interface::SystemInterface)
