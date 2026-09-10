// Copyright (c) 2024 Open Navigation LLC
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

// humble 백포트 (jazzy 1.3.12 nav2_behavior_tree 에서 이식, BT.CPP v3 헤더만 교체).
// alice_navigation factory_nav_to_pose.humble.xml 이 first/last mile 경로를 route 경로에 이어 붙이는 데 쓴다.

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONCATENATE_PATHS_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONCATENATE_PATHS_ACTION_HPP_

#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to concatenate two paths into one
 */
class ConcatenatePaths : public BT::ActionNodeBase
{
public:
  ConcatenatePaths(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path1", "Input Path 1 to cancatenate"),
      BT::InputPort<nav_msgs::msg::Path>("input_path2", "Input Path 2 to cancatenate"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Paths concatenated"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__CONCATENATE_PATHS_ACTION_HPP_
