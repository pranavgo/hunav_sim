#include "hunav_agent_manager/extended_agent_manager.hpp"

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "hunav_msgs/msg/agent.hpp"
#include "hunav_msgs/msg/agents.hpp"

// Behavior Trees
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
//#ifdef ZMQ_FOUND
//#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
//#endif

#include <iostream>
//#include <memory>
#include <chrono>
#include <math.h> /* fabs */
#include <mutex>
#include <string>

// Social Force Model
#include <lightsfm/sfm.hpp>

namespace hunav {

class BTfunctions {
public:
  /**
   * @brief Construct a new BTfunctions object
   *
   */
  BTfunctions();
  /**
   * @brief Destroy the BTfunctions object
   *
   */
  ~BTfunctions();

  void init();

  void updateAllAgents(const hunav_msgs::msg::Agent::SharedPtr robot,
                       const hunav_msgs::msg::Agents::SharedPtr msg) {
    agent_manager_.updateAllAgents(robot, msg);
  }

  void updateAgentsAndRobot(const hunav_msgs::msg::Agents::SharedPtr msg) {
    agent_manager_.updateAgentsAndRobot(msg);
  }

  hunav_msgs::msg::Agents getUpdatedAgents() {
    
    return agent_manager_.getUpdatedAgentsMsg();
  }
  hunav_msgs::msg::Agent getUpdatedAgent(int id) {
    return agent_manager_.getUpdatedAgentMsg(id);
  }

  // bool running();
  // bool ok() { return agent_manager_.canCompute(); }

  sfm::Forces getAgentForces(int id) {
    return agent_manager_.getAgentForces(id);
  };

  // BT Conditions
  BT::NodeStatus robotVisible(BT::TreeNode &self);
  BT::NodeStatus robotNearby(BT::TreeNode &self);
  BT::NodeStatus robotSays(BT::TreeNode &self);
  BT::NodeStatus humanSays(BT::TreeNode &self);
  BT::NodeStatus goalReached(BT::TreeNode &self);
  BT::NodeStatus makeGesture(BT::TreeNode &self);
  BT::NodeStatus robotBlocking(BT::TreeNode &self);

  //<NEW BT CONDITION>


  // BT Actions to be registered with SimpleActionNode
  BT::NodeStatus updateGoal(BT::TreeNode &self);
  BT::NodeStatus regularNav(BT::TreeNode &self);
  BT::NodeStatus lookAtRobot(BT::TreeNode &self);
  BT::NodeStatus robotMoved(BT::TreeNode &self);
  BT::NodeStatus avoidRobot(BT::TreeNode &self);
  BT::NodeStatus givewaytoRobot(BT::TreeNode &self);
  BT::NodeStatus followRobot(BT::TreeNode &self);
  BT::NodeStatus followHuman(BT::TreeNode &self);
  BT::NodeStatus blockRobot(BT::TreeNode &self);

  //<NEW BT ACTION>

protected:
  AgentManagerExt agent_manager_;
};

} // namespace hunav
