// Copyright 2020 Intelligent Robotics Lab
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

#ifndef manipulator_bt__BEHAVIOR_TREE_NODES__PLACE_HPP_
#define manipulator_bt__BEHAVIOR_TREE_NODES__PLACE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

//#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "mros2_msgs/action/place_qos.hpp"

#include "manipulator_bt/BTActionNode.hpp"
#include "rclcpp/rclcpp.hpp"

using PlaceQoS = mros2_msgs::action::PlaceQos;

namespace manipulator_bt
{

class Place : public BtActionNode<PlaceQoS>
{
public:
  explicit Place(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  void on_wait_for_result() override;

  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::string>("goal") 
    };
  }
private:
  rclcpp::Node::SharedPtr node_;
 // geometry_msgs::msg::Pose wp_;
};

}  // namespace manipulator_bt

#endif  // manipulator_bt__BEHAVIOR_TREE_NODES__PLACE_HPP_
