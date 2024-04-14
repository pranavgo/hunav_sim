#include "hunav_agent_manager/agent_manager.hpp"

namespace hunav
{

  AgentManager::AgentManager()
  {
    init();
    printf("[AgentManager.Constructor] AgentManager initialized \n");
  }

  AgentManager::~AgentManager() {}

  void AgentManager::init()
  {
    agents_initialized_ = false;
    robot_initialized_ = false;
    agents_received_ = false;
    robot_received_ = false;
    max_dist_view_ = 10.0;
    time_step_secs_ = 0.0;
    step_count = 1;
    step_count2 = 1;
    move = false;
    printf("[AgentManager.init] initialized \n");
  }

  // Existing functions remain unchanged

  void AgentManager::helloWorld()
  {
    printf("Hello World!\n");
  }

}

