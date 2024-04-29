#include "hunav_agent_manager/bt_functions.hpp"

namespace hunav
{

  BTfunctions::BTfunctions()
  {
    init();
    printf("[BTfunctions.Constructor] initialized!\n");
  }

  BTfunctions::~BTfunctions() {}

  void BTfunctions::init()
  {
    printf("[BTfunctions.init] initialized!\n");
  }
  
  BT::NodeStatus BTfunctions::robotSays(BT::TreeNode &self){
    auto idmsg = self.getInput<int>("agent_id");
    auto msg = self.getInput<int>("msg");
    if (!idmsg)
    {
      throw BT::RuntimeError("RobotSays. missing required input [agent_id]: ", idmsg.error());
    }
    if (!msg)
    {
      throw BT::RuntimeError("RobotSays. missing required input [msg]: ", msg.error());
    }
    
    int id = idmsg.value();
    int message = msg.value();
    if (agent_manager_.robotSays(id,message)){
      return BT::NodeStatus::SUCCESS;
    }
    else{
      return BT::NodeStatus::FAILURE;
    }
  }
  //check if the robot is visible to an agent
  BT::NodeStatus BTfunctions::robotVisible(BT::TreeNode &self)
  {
    auto idmsg = self.getInput<int>("agent_id");
    if (!idmsg)
    {
      throw BT::RuntimeError("RobotVisible. missing required input [agent_id]: ",
                             idmsg.error());
    }
    auto dmsg = self.getInput<double>("distance");
    if (!dmsg)
    {
      throw BT::RuntimeError("RobotVisible. missing required input [distance]: ",
                             dmsg.error());
    }

    int id = idmsg.value();
    double dist = dmsg.value();
    //std::cout << "BTfunctions.robotVisible. Ticking agent: " << id << std::endl;
    if (agent_manager_.isRobotVisible(id, dist))
    {
      std::cout << "BTfunctions.robotVisible. Returning success" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      std::cout << "BTfunctions.robotVisible. Returning failure" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
  }

  //check if an agent has reached their goal
  BT::NodeStatus BTfunctions::goalReached(BT::TreeNode &self)
  {
    auto msg = self.getInput<int>("agent_id");
    if (!msg)
    {
      throw BT::RuntimeError("GoalReached. missing required input [agent_id]: ",msg.error());
    }
    int id = msg.value();
    if (agent_manager_.goalReached(id))
    {
      std::cout << "BTfunctions.GoalReached. agent: " << id << " Goal Reached!" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      //std::cout << "BTfunctions.GoalReached. agent: " << id << " Goal not reached"<< std::endl;
      return BT::NodeStatus::FAILURE;
    }
  }

  //update the goal of an agent by popping the top goal 
  BT::NodeStatus BTfunctions::updateGoal(BT::TreeNode &self)
  {
    auto msg = self.getInput<int>("agent_id");
    if (!msg)
    {
      throw BT::RuntimeError("UpdateGoal. missing required input [agent_id]: ",
                             msg.error());
    }
    int id = msg.value();
    if (agent_manager_.updateGoal(id))
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }

  //make agent do a gesture. Valid gestures: {0: NONE, 1: WAIT, 2: PROCEED}
  BT::NodeStatus BTfunctions::makeGesture(BT::TreeNode &self)
  {
    auto msg = self.getInput<int>("agent_id");
    if (!msg)
    {
      throw BT::RuntimeError("MakeGesture. missing required input [agent_id]: ",
                             msg.error());
    }
    auto gmsg = self.getInput<double>("message");
    if (!gmsg)
    {
      throw BT::RuntimeError("makeGesture. missing required input [message]: ",
                             gmsg.error());
    }

    int id = msg.value();
    double message = gmsg.value();
    std::cout << "BTfunctions.makeGesture. Messaging: " << message << std::endl;
    agent_manager_.makeGesture(id,message);
    return BT::NodeStatus::SUCCESS;
  }

  //Regular Navigation
  BT::NodeStatus BTfunctions::regularNav(BT::TreeNode &self)
  {

    auto msg = self.getInput<int>("agent_id");
    auto msg2 = self.getInput<double>("time_step");
    if (!msg)
    {
      throw BT::RuntimeError("RegularNav. missing required input [agent_id]: ",
                             msg.error());
    }
    if (!msg2)
    {
      throw BT::RuntimeError("RegularNav. missing required input [time_step]: ",
                             msg2.error());
    }

    int id = msg.value();
    double dt = msg2.value();
    
    // Update SFM model position
    std::cout << "BTfunctions.RegularNav. Ticking agent" << id <<":  "<< dt << std::endl;
    agent_manager_.updatePosition(id, dt); //update the position of an agent in the simulator

    
    return BT::NodeStatus::SUCCESS;
  }

  //Surprised Navigation
  BT::NodeStatus BTfunctions::surprisedNav(BT::TreeNode &self)
  {
    auto msg = self.getInput<int>("agent_id");
    if (!msg)
    {
      throw BT::RuntimeError("SurprisedNav. missing required input [agent_id]: ",
                             msg.error());
    }
    int id = msg.value();
    // stop the agent and just look at the robot (change the agent orientation)
    std::cout<< "BTfunctions.SurprisedNav. Ticking agent" << id << std::endl;
    agent_manager_.lookAtTheRobot(id);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus BTfunctions::curiousNav(BT::TreeNode &self)
  {
    auto msg = self.getInput<int>("agent_id");
    auto msg2 = self.getInput<double>("time_step");
    if (!msg)
    {
      throw BT::RuntimeError("CuriousNav. missing required input [agent_id]: ",
                             msg.error());
    }
    if (!msg2)
    {
      throw BT::RuntimeError("CuriousNav. missing required input [time_step]: ",
                             msg2.error());
    }
    int id = msg.value();
    double dt = msg2.value();

    agent_manager_.approximateRobot(id, dt);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus BTfunctions::scaredNav(BT::TreeNode &self)
  {
    auto msg = self.getInput<int>("agent_id");
    auto msg2 = self.getInput<double>("time_step");
    if (!msg)
    {
      throw BT::RuntimeError("ScaredNav. missing required input [agent_id]: ",
                             msg.error());
    }
    if (!msg2)
    {
      throw BT::RuntimeError("ScaredNav. missing required input [time_step]: ",
                             msg2.error());
    }
    int id = msg.value();
    double dt = msg2.value();

    agent_manager_.avoidRobot(id, dt);
    return BT::NodeStatus::SUCCESS;
  }
  
  BT::NodeStatus BTfunctions::threateningNav(BT::TreeNode &self)
  {
    auto msg = self.getInput<int>("agent_id");
    auto msg2 = self.getInput<double>("time_step");
    if (!msg)
    {
      throw BT::RuntimeError(
          "threateningNav. missing required input [agent_id]: ", msg.error());
    }
    if (!msg2)
    {
      throw BT::RuntimeError(
          "threateningNav. missing required input [time_step]: ", msg2.error());
    }
    int id = msg.value();
    double dt = msg2.value();

    agent_manager_.blockRobot(id, dt);
    return BT::NodeStatus::SUCCESS;
  }
}
