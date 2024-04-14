#ifndef AGENT_MANAGER_HPP
#define AGENT_MANAGER_HPP

#include "hunav_msgs/msg/agents.hpp"
#include "hunav_msgs/msg/agent.hpp"
#include "sfm/sfm.hpp"
#include "utils/utils.hpp"

#include <mutex>
#include <unordered_map>
#include <list>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace hunav
{
  class AgentManager
  {
  public:
    AgentManager();
    ~AgentManager();

    void init();
    float robotSquaredDistance(int id);
    bool lineOfSight(int id);
    bool isRobotVisible(int id, double dist);
    utils::Vector2d getRobotVelocity();
    void lookAtTheRobot(int id);
    void approximateRobot(int id, double dt);
    void blockRobot(int id, double dt);
    void makeGesture(int id, int gesture = 0);
    utils::Vector2d getRobotPosition();
    void avoidRobot(int id, double dt);
    bool goalReached(int id);
    bool updateGoal(int id);
    void updatePosition(int id, double dt);
    void initializeAgents(const hunav_msgs::msg::Agents::SharedPtr msg);
    void initializeRobot(const hunav_msgs::msg::Agent::SharedPtr msg);
    bool updateAgents(const hunav_msgs::msg::Agents::SharedPtr msg);
    void updateAgentRobot(const hunav_msgs::msg::Agent::SharedPtr msg);
    hunav_msgs::msg::Agent getUpdatedAgentMsg(int id);
    hunav_msgs::msg::Agents getUpdatedAgentsMsg();
    std::vector<sfm::Agent> getSFMAgents();
    void computeForces(int id);
    void computeForces();
    void updateAllAgents(const hunav_msgs::msg::Agent::SharedPtr robot_msg, const hunav_msgs::msg::Agents::SharedPtr agents_msg);
    void updateAgentsAndRobot(const hunav_msgs::msg::Agents::SharedPtr agents_msg);
    bool canCompute();
    
    // New function to print "hello world"
    void helloWorld();

  private:
    // Member variables and other functions remain unchanged
  };
}

#endif
