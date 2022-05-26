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

/* Author: Lorena Bajo Rebollo lorena.bajo@urjc.es */

/* Mantainer: Jonatan Gines Clavero jonatan.gines@urjc.es */


#include <math.h>
#include <iostream>
#include <memory>
#include <string>
#include <map>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/blackboard.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string pkgpath = ament_index_cpp::get_package_share_directory("manipulator_bt");
  std::string xml_file = pkgpath + "/behavior_trees/bt_mc.xml";

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;
  factory.registerFromPlugin(loader.getOSName("manipulator_bt_navigate_to_wp_bt_node"));
  factory.registerFromPlugin(loader.getOSName("manipulator_bt_reconfigure_bt_node"));
  factory.registerFromPlugin(loader.getOSName("manipulator_bt_check_component_bt_node"));
  factory.registerFromPlugin(loader.getOSName("manipulator_bt_pick_bt_node"));
  factory.registerFromPlugin(loader.getOSName("manipulator_bt_place_bt_node"));

  auto blackboard = BT::Blackboard::create();
  auto node = rclcpp::Node::make_shared("pilot_node");
  auto graph = std::make_shared<ros2_knowledge_graph::GraphNode>("pilot_graph");
  graph->start();
  blackboard->set("node", node);
  blackboard->set("pilot_graph", graph);  

  std::unordered_map<std::string, geometry_msgs::msg::Pose> wp_map;
  geometry_msgs::msg::Pose wp;
  wp.position.x = 1.1;  
  wp.position.y = -0.15; 
  wp.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0);
  wp_map.insert(std::pair<std::string, geometry_msgs::msg::Pose>("wp_1", wp));

  wp.position.x = -0.5;
  wp.position.y = -1.1;
  wp.orientation = nav2_util::geometry_utils::orientationAroundZAxis(-M_PI/2);
  wp_map.insert(std::pair<std::string, geometry_msgs::msg::Pose>("wp_2", wp));

  wp.position.x = 1.0;
  wp.position.y = 0.25;
  wp.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0);
  wp_map.insert(std::pair<std::string, geometry_msgs::msg::Pose>("wp_3", wp));

  wp.position.x = -0.5;
  wp.position.y = -1.15;
  wp.orientation = nav2_util::geometry_utils::orientationAroundZAxis(-M_PI/2);
  wp_map.insert(std::pair<std::string, geometry_msgs::msg::Pose>("wp_4", wp));

  blackboard->set("wp_map", wp_map); 

  // ADD PICK & PLACE MAPS
// <Action ID="Pick" aruco_id="5" object_type="hagelslag"/>
// <Action ID="Place" aruco_id="333"/>
// <Action ID="Pick" aruco_id="12" object_type="hagelslag"/>
// <Action ID="Place" aruco_id="222"/>
 

  // Create blackboard for component status
  std::unordered_map<std::string, bool> component_map;

  component_map.insert(std::pair<std::string, bool>("battery", true));
  component_map.insert(std::pair<std::string, bool>("camera", true));
  component_map.insert(std::pair<std::string, bool>("laser", true));
  component_map.insert(std::pair<std::string, bool>("left_arm", true));
  component_map.insert(std::pair<std::string, bool>("right_arm", true));
  blackboard->set("component_map", component_map); 

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  rclcpp::Rate rate(1.0);
  bool finished = false;
  while (rclcpp::ok() && !finished) {
    finished = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Pilot execution finished");

  rclcpp::shutdown();

  return 0;
}
