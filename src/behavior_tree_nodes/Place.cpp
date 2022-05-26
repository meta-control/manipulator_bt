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

#include <string>
#include <memory>

#include "manipulator_bt/behavior_tree_nodes/Place.hpp"


namespace manipulator_bt
{

Place::Place(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<PlaceQoS>(xml_tag_name, action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

void Place::on_tick()
{
  auto wp_map = 
 // PICKING LOGGERS, NAV EXAMPLE
        // config().blackboard->get<std::unordered_map<std::string, geometry_msgs::msg::Pose>>("wp_map");
        // wp_ = wp_map[res];
        // RCLCPP_INFO(node_->get_logger(), "Navigating to... [%s -- %f %f]",
        // res.c_str(), wp_.position.x, wp_.position.y);
        // goal_.pose.pose.position = wp_.position;
        // goal_.pose.pose.orientation = wp_.orientation;

  goal_.qos_expected.objective_type = "f_place"; // should be mros_goal->qos_expected.objective_type = "f_place";
  diagnostic_msgs::msg::KeyValue reliability_qos;
  reliability_qos.key = "reliability";
  reliability_qos.value = "0.5";
  goal_.qos_expected.qos.clear();
  goal_.qos_expected.qos.push_back(reliability_qos);
}

void Place::on_wait_for_result()
{  
        // PLACE CHECK, NAV EXAMPLE
                // std::string goal_id = rclcpp_action::to_string(goal_handle_->get_goal_id()); 
                // if (!goal_id.compare(feedback_->qos_status.objective_id) == 0){

                // RCLCPP_INFO(node_->get_logger(), "goal id and feedback are different");
                // rclcpp::Rate(1).sleep(); //  Wait for the goal to finish
                // return;
                // }
                // RCLCPP_INFO(node_->get_logger(), "Curr mode: %s ", feedback_->qos_status.selected_mode.c_str()); 
                

  // check selected_mode

   if (feedback_->qos_status.selected_mode == "fd_place_left_mode")
  {
    RCLCPP_ERROR(node_->get_logger(), "Right arm failed");
    auto component_map =
      config().blackboard->get<std::unordered_map<std::string, bool>>("component_map");
    
    auto right_arm_component = component_map.find("right_arm");
    if (right_arm_component->second)
    {
      RCLCPP_ERROR(node_->get_logger(), "Set right arm to false");
      right_arm_component->second = false;
      halt();
      config().blackboard->set("component_map", component_map); 
      result_.code = rclcpp_action::ResultCode::ABORTED;
      goal_result_available_ = true;
    }
  }

  else if (feedback_->qos_status.selected_mode == "fd_place_right_mode")
  {
    RCLCPP_ERROR(node_->get_logger(), "Left arm failed");
    auto component_map =
      config().blackboard->get<std::unordered_map<std::string, bool>>("component_map");
    
    auto left_arm_component = component_map.find("left_arm");
    if (left_arm_component->second)
    {
      RCLCPP_ERROR(node_->get_logger(), "Set left arm to false");
      left_arm_component->second = false;
      halt();
      config().blackboard->set("component_map", component_map); 
      result_.code = rclcpp_action::ResultCode::ABORTED;
      goal_result_available_ = true;
    }
  }

}

BT::NodeStatus Place::on_success()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace manipulator_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<manipulator_bt::Place>(
        name, "place_qos", config);
    };

  factory.registerBuilder<manipulator_bt::Place>(
    "Place", builder);
}
