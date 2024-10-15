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
  float AgentManager::robotSquaredDistance(int id)
  {
    float xa = agents_[id].sfmAgent.position.getX(); // position.position.x;
    float ya = agents_[id].sfmAgent.position.getY(); // position.y;
    float xr = robot_.sfmAgent.position.getX();      // position.x;
    float yr = robot_.sfmAgent.position.getY();      // position.y;
    return (xr - xa) * (xr - xa) + (yr - ya) * (yr - ya);
  }
  float AgentManager::SquaredDistance(int id, int target_id)
  {
    float xa = agents_[id].sfmAgent.position.getX(); // position.position.x;
    float ya = agents_[id].sfmAgent.position.getY(); // position.y;
    float xr = agents_[target_id].sfmAgent.position.getX();     // position.x;
    float yr = agents_[target_id].sfmAgent.position.getY();     // position.y;
    return (xr - xa) * (xr - xa) + (yr - ya) * (yr - ya);
  }

  bool AgentManager::lineOfSight(int id, double tolerance)
  {
    float ax = agents_[id].sfmAgent.position.getX();
    float ay = agents_[id].sfmAgent.position.getY();
    float rx = robot_.sfmAgent.position.getX();
    float ry = robot_.sfmAgent.position.getY();
    double yaw = agents_[id].sfmAgent.yaw.toRadian();
    float nrx = (rx - ax) * cos(yaw) + (ry - ay) * sin(yaw);
    float nry = -(rx - ax) * sin(yaw) + (ry - ay) * cos(yaw);
    float rangle = atan2(nry, nrx);
    //std::cout<<"YOUR ANGLE IS"<<rangle<<std::endl;


    if (abs(rangle) > (tolerance))
    {
      return false;
    }
    else
    {
      //std::cout<<"robot is in line of sight";
      return true;
    }
  }

  bool AgentManager::isRobotVisible(int id, double dist, double tolerance)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    float squared_dist = robotSquaredDistance(id);
    //std::cout<<"Robot Distance: "<<sqrt(squared_dist)<<"Reqd distance:"<<dist<<std::endl;
    if (sqrt(squared_dist) <= (dist))
    {
      return lineOfSight(id, tolerance);
    }
    else
    {
      return false;
    }
  }


  bool AgentManager::isRobotNearby(int id, double dist)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    float squared_dist = robotSquaredDistance(id);
    //std::cout<<"Robot Distance: "<<squared_dist<<std::endl;
    if (sqrt(squared_dist) <= (dist))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  
  utils::Vector2d AgentManager::getRobotVelocity()
  {
    std::lock_guard<std::mutex> guard(mutex_);
    // Assuming robot_ has velocity attributes vx and vy
    return utils::Vector2d(robot_.sfmAgent.velocity.getX(), robot_.sfmAgent.velocity.getY());
  }

  bool AgentManager::hasRobotMoved(int id)
  {

    std::lock_guard<std::mutex> guard(mutex_);
    // Robot position
    if (robot_.sfmAgent.velocity.getX()<0.1 && robot_.sfmAgent.velocity.getY()<0.1){
      return false;
    }
    return true;
    
  }
  
  void AgentManager::lookAtRobot(int id)
  {
    agents_[id].behavior_state = 1;
    std::lock_guard<std::mutex> guard(mutex_);
    // Robot position
    float rx = robot_.sfmAgent.position.getX();
    float ry = robot_.sfmAgent.position.getY();
    // Agent position
    float ax = agents_[id].sfmAgent.position.getX();
    float ay = agents_[id].sfmAgent.position.getY();
    float ah = agents_[id].sfmAgent.yaw.toRadian();
    // Transform robot position to agent coords system
    float nrx = (rx - ax) * cos(ah) + (ry - ay) * sin(ah);
    float nry = -(rx - ax) * sin(ah) + (ry - ay) * cos(ah);
    utils::Angle robotYaw; // = utils::Angle::fromRadian(atan2(nry, nrx));
    robotYaw.setRadian(atan2(nry, nrx));
    float max_ang_vel = M_PI; // rad/secs
    utils::Angle max_angle =
        utils::Angle::fromRadian(max_ang_vel * 0.01);
    if (robotYaw.sign() < 0)
      max_angle.setRadian(max_angle.toRadian() * (-1));

    // Update the agent angle
    if (fabs(robotYaw.toRadian()) > max_angle.toRadian())
    {
      agents_[id].sfmAgent.yaw = (agents_[id].sfmAgent.yaw + max_angle);
    }
    else
    {
      agents_[id].sfmAgent.yaw = agents_[id].sfmAgent.yaw + robotYaw;
    }
    
  }
  
  void AgentManager::followRobot(int id, double dt)
  {

    std::lock_guard<std::mutex> guard(mutex_);

    agents_[id].behavior_state = 1;

    // Robot position
    float rx = robot_.sfmAgent.position.getX();
    float ry = robot_.sfmAgent.position.getY();
    float dist = sqrt(robotSquaredDistance(id));

    // if the agent is close to the robot,
    // stop and look at the robot
    if (dist <= 1.5)  // 1.5
    {
      // printf("Agent %i stoping and looking at the robot! dist: %.2f\n", id,
      // dist);
      // Agent position
      float ax = agents_[id].sfmAgent.position.getX();
      float ay = agents_[id].sfmAgent.position.getY();
      float ah = agents_[id].sfmAgent.yaw.toRadian();
      // Transform robot position to agent coords system
      float nrx = (rx - ax) * cos(ah) + (ry - ay) * sin(ah);
      float nry = -(rx - ax) * sin(ah) + (ry - ay) * cos(ah);
      utils::Angle Yaw;  // = utils::Angle::fromRadian(atan2(nry, nrx));
      Yaw.setRadian(atan2(nry, nrx));

      agents_[id].sfmAgent.yaw = agents_[id].sfmAgent.yaw + Yaw;
    }
    else{
      // Change the agent goal
      sfm::Goal g;
      g.center.set(rx, ry);
      g.radius = robot_.sfmAgent.radius;
      agents_[id].sfmAgent.goals.push_front(g);

      // change agent vel according to the proximity of the robot
      // move slowly when close
      float ini_desired_vel = agents_[id].sfmAgent.desiredVelocity;
      agents_[id].sfmAgent.desiredVelocity = 1.8 * (dist / max_dist_view_);
      // recompute forces
      computeForces(id);
      // update position
      sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
      // restore values just in case the approximation
      // ends in the next iteration
      agents_[id].sfmAgent.goals.pop_front();
      agents_[id].sfmAgent.desiredVelocity = ini_desired_vel;
    }
  }
  
  void AgentManager::followHuman(int id, int target_id, double dt)
  {

    std::lock_guard<std::mutex> guard(mutex_);
    
    agents_[id].behavior_state = 1;

    // human position
    float rx = agents_[target_id].sfmAgent.position.getX();
    float ry =  agents_[target_id].sfmAgent.position.getY();
    float dist = sqrt(SquaredDistance(id,target_id));

    // if the agent is close to the robot,
    // stop and look at the robot
    if (dist <= 1.5)  // 1.5
    {
      // printf("Agent %i stoping and looking at the robot! dist: %.2f\n", id,
      // dist);
      // Agent position
      float ax = agents_[id].sfmAgent.position.getX();
      float ay = agents_[id].sfmAgent.position.getY();
      float ah = agents_[id].sfmAgent.yaw.toRadian();
      // Transform robot position to agent coords system
      float nrx = (rx - ax) * cos(ah) + (ry - ay) * sin(ah);
      float nry = -(rx - ax) * sin(ah) + (ry - ay) * cos(ah);
      utils::Angle Yaw;  // = utils::Angle::fromRadian(atan2(nry, nrx));
      Yaw.setRadian(atan2(nry, nrx));

      agents_[id].sfmAgent.yaw = agents_[id].sfmAgent.yaw + Yaw;
    }
    else{
      // Change the agent goal
      sfm::Goal g;
      g.center.set(rx, ry);
      g.radius = agents_[target_id].sfmAgent.radius;
      agents_[id].sfmAgent.goals.push_front(g);
      int num_goals = agents_[id].sfmAgent.goals.size();
      std::cout<<"-------------------"<<agents_[id].sfmAgent.goals.size()<<std::endl;
      // change agent vel according to the proximity of the robot
      // move slowly when close
      float ini_desired_vel = agents_[id].sfmAgent.desiredVelocity;
      agents_[id].sfmAgent.desiredVelocity = 1.8 * (dist / max_dist_view_);
      //std::cout<<"-------------------"<<agents_[id].sfmAgent.desiredVelocity<<std::endl;
      // recompute forces
      computeForces(id);
      // update position
      sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
      // restore values just in case the approximation
      // ends in the next iteration
      
      if(agents_[id].sfmAgent.goals.size() == num_goals)
      { 
        agents_[id].sfmAgent.goals.pop_front();
        std::cout<<"-------------------"<<agents_[id].sfmAgent.goals.size()<<std::endl;
      }
      agents_[id].sfmAgent.desiredVelocity = ini_desired_vel;
      std::cout<<"-----------Follow Human--------"<<agents_[id].sfmAgent.desiredVelocity<<std::endl;
    }
  }

  void AgentManager::blockRobot(int id, double dt)
  {

    std::lock_guard<std::mutex> guard(mutex_);
    agents_[id].behavior_state = 1;

    // Robot position
    float rx = robot_.sfmAgent.position.getX();
    float ry = robot_.sfmAgent.position.getY();

    float h = robot_.sfmAgent.yaw.toRadian();

    // Store the initial set o goals
    std::list<sfm::Goal> gls = agents_[id].sfmAgent.goals;

    // Change the agent goal.
    // We should compute a goal in front of the robot heading.
    float newgx = rx + 1.5 * cos(h);
    float newgy = ry + 1.5 * sin(h);
    // std::cout<<"Goal:"<<newgx<<","<<newgy<<std::endl;

    sfm::Goal g;
    g.center.set(newgx, newgy);
    g.radius = 0.05; // robot_.sfmAgent.radius;
    agents_[id].sfmAgent.goals.push_front(g);

    // change agent vel according to the proximity of the robot
    // move slowly when close
    float ini_desired_vel = agents_[id].sfmAgent.desiredVelocity;
    agents_[id].sfmAgent.desiredVelocity = 0.5;
    // recompute forces
    computeForces(id);
    // update position
    sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
    // if the agent is close to the robot,
    // look at the robot
    float dist = sqrt(robotSquaredDistance(id));
    if (dist <= 0.6)
    {
      // std::cout<<"Robot close"<<std::endl;
      // Agent position
      float ax = agents_[id].sfmAgent.position.getX();
      float ay = agents_[id].sfmAgent.position.getY();
      float ah = agents_[id].sfmAgent.yaw.toRadian();
      // Transform robot position to agent coords system
      float nrx = (rx - ax) * cos(ah) + (ry - ay) * sin(ah);
      float nry = -(rx - ax) * sin(ah) + (ry - ay) * cos(ah);
      utils::Angle robotYaw; // = utils::Angle::fromRadian(atan2(nry, nrx));
      robotYaw.setRadian(atan2(nry, nrx));
      agents_[id].sfmAgent.yaw = agents_[id].sfmAgent.yaw + robotYaw;
    }

    // restore the goals and the velocity
    agents_[id].sfmAgent.goals = gls;
    agents_[id].sfmAgent.desiredVelocity = ini_desired_vel;
    std::cout<<"BTFunctions.Block Robot Ticking agent"<<std::endl;
  }
  
  void AgentManager::makeGesture(int id, int gesture = 1)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    //std::cout<< "Agent " << id << " is making gesture " << gesture << std::endl;
    agents_[id].gesture = gesture; 
  }

  utils::Vector2d AgentManager::getRobotPosition()
  {
    std::lock_guard<std::mutex> guard(mutex_);
    return utils::Vector2d(robot_.sfmAgent.position.getX(), robot_.sfmAgent.position.getY());
  }
  
  void AgentManager::givewaytoRobot(int id, double dt)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (agents_[id].behavior_state!=2){
      agents_[id].behavior_state = 2;

      //robot position
      float rx = robot_.sfmAgent.position.getX();
      float ry = robot_.sfmAgent.position.getY();
      float h = robot_.sfmAgent.yaw.toRadian();
      float agx = agents_[id].sfmAgent.position.getX();
      float agy = agents_[id].sfmAgent.position.getY();
      // Store the initial set o goals
      std::list<sfm::Goal> gls = agents_[id].sfmAgent.goals;

      // Change the agent goal.
      // We should compute a goal out of the robot's direct path, (assumed to be along the line connecting the robot and the human)
      float alpha = 2.0;
      float r = sqrt(pow((rx - agx),2) + pow((ry - agy),2));
      float newgx = alpha*(agy-ry)/r;
      float newgy = alpha*(agx-rx)/r;
      
      sfm::Goal g;
      g.center.set(newgx, newgy);
      g.radius = 0.05; // robot_.sfmAgent.radius;
      agents_[id].sfmAgent.goals.push_front(g);
      agents_[id].sfmAgent.goals = gls;
    }
    //std::cout<<"BTFunctions.giveway to Robot Ticking agent"<<std::endl;
    computeForces(id);
    sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
  
  }

  void AgentManager::avoidRobot(int id, double dt)
  {
    std::lock_guard<std::mutex> guard(mutex_);

    agents_[id].behavior_state = 1;

    // we decrease the maximum velocity
    double init_vel = agents_[id].sfmAgent.desiredVelocity;
    agents_[id].sfmAgent.desiredVelocity = 0.6;
    computeForces(id);

    // We add an extra repulsive force from the robot
    utils::Vector2d minDiff =
        agents_[id].sfmAgent.position - robot_.sfmAgent.position;

    double distance = minDiff.norm() - agents_[id].sfmAgent.radius;

    utils::Vector2d Scaryforce =
        20.0 * (agents_[id].sfmAgent.params.forceSigmaObstacle / distance) *
        minDiff.normalized();
    
    agents_[id].sfmAgent.forces.globalForce += Scaryforce;

    // update position
    sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);

    // restore desired vel
    agents_[id].sfmAgent.desiredVelocity = init_vel;
  }

  bool AgentManager::goalReached(int id)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    // if (agents_[id].sfmAgent.goals.empty()){
    //   std::cout<<"~~~~~~~~~~GOAL REACHED IS TRUE AND LIST IS EMPTY~~~~~~~~~~~~~~"<<std::endl;
    //   for (auto b : agents_[id].sfmAgent.goals){
    //     std::cout<<"GOAL:"<< b.center.getX()<<"_"<< b.center.getY();
    //   }
    //   std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;
    //   // printf("Agent %s goal reached!", agents_[id].name.c_str());
    //   return true;
    //}

    if (!agents_[id].sfmAgent.goals.empty() &&
        (agents_[id].sfmAgent.goals.front().center -
         agents_[id].sfmAgent.position)
                .norm() <= (agents_[id].sfmAgent.goals.front().radius + 0.1))
    {
      // printf("Agent %s goal reached!", agents_[id].name.c_str());
      return true;
    }
    else{
      return false;
    }
  }

  bool AgentManager::robotSays(int id, int msg){
    std::lock_guard<std::mutex> guard(mutex_);
    if (robot_.gesture == msg){
      return true;
    }
    else
      return false;
  }

  bool AgentManager::humanSays(int id,int target_id,int msg){
    std::lock_guard<std::mutex> guard(mutex_);
    //std::cout<<"Pinging Human Says"<<"\n";
    if (agents_[target_id].gesture == msg){
      return true;
    }
    else
      return false;
  }


  bool AgentManager::updateGoal(int id)
  {
    std::lock_guard<std::mutex> guard(mutex_);

    //printf("Updating goal for agent %i\n\n", id); 
    sfm::Goal g = agents_[id].sfmAgent.goals.front();    
    if (agents_[id].sfmAgent.goals.size() != 1){
      agents_[id].sfmAgent.goals.pop_front();
      //agents_[id].current_goal_index+=1;
    }

    //std::cout<<"Next Goal:"<<agents_[id].sfmAgent.goals.back().center.getX()<<","<<agents_[id].sfmAgent.goals.back().center.getY()<<"\n";

    if (agents_[id].sfmAgent.cyclicGoals)
    {
      agents_[id].sfmAgent.goals.push_back(g);    }  
    return true;
  }
  
  void AgentManager::updatePosition(int id, double dt)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    agents_[id].behavior_state = 0;
    // check if pause_nav is true for this agent and pause navigation accordingly
    if(agents_[id].pause_nav==false){
        sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
    }
  }

  void AgentManager::initializeAgents(
      const hunav_msgs::msg::Agents::SharedPtr msg)
  {

    printf("Initializing SFM Agents...\n");
    for (auto a : msg->agents)
    {
      agent ag;
      ag.name = a.name;
      ag.type = a.type;
      ag.behavior = a.behavior;
      ag.pause_nav = false;
      //ag.current_goal_index = 0;
      ag.behavior_state = a.behavior_state;
      ag.sfmAgent.id = a.id;
      ag.sfmAgent.groupId = a.group_id;
      ag.sfmAgent.desiredVelocity = a.desired_velocity;
      ag.sfmAgent.radius = a.radius;
      ag.sfmAgent.cyclicGoals = a.cyclic_goals;
      ag.sfmAgent.position.set(a.position.position.x, a.position.position.y);
      ag.sfmAgent.yaw.setRadian(a.yaw);
      ag.sfmAgent.velocity.set(a.velocity.linear.x, a.velocity.linear.y);
      ag.sfmAgent.linearVelocity =
          sqrt(a.velocity.linear.x * a.velocity.linear.x +
               a.velocity.linear.y * a.velocity.linear.y);
      ag.sfmAgent.angularVelocity = a.velocity.angular.z;
      for (auto g : a.goals)
      {
        sfm::Goal sfmg;
        sfmg.center.setX(g.position.x);
        sfmg.center.setY(g.position.y);
        sfmg.radius = a.goal_radius;
        ag.sfmAgent.goals.push_back(sfmg);
      }
      ag.sfmAgent.obstacles1.clear();
      if (!a.closest_obs.empty())
      {
        for (auto obs : a.closest_obs)
        {
          utils::Vector2d o;
          o.set(obs.x, obs.y);
          ag.sfmAgent.obstacles1.push_back(o);
        }
      }
      ag.sfmAgent.params.forceFactorSocial = 5.0; // 40.0; // 2.1 by default
      //  ag.sfmAgent.params.forceFactorDesired = 5.0;
      //  ag.sfmAgent.params.forceFactorObstacle = 20.0;

      agents_[ag.sfmAgent.id] = ag;
      printf("\tagent %s, x:%.2f, y:%.2f, th:%.2f\n",
             agents_[ag.sfmAgent.id].name.c_str(),
             agents_[ag.sfmAgent.id].sfmAgent.position.getX(),
             agents_[ag.sfmAgent.id].sfmAgent.position.getY(),
             agents_[ag.sfmAgent.id].sfmAgent.yaw.toRadian());
      for (auto g : agents_[ag.sfmAgent.id].sfmAgent.goals){
        printf("\t\tgoal x:%.2f, y:%.2f\n", g.center.getX(), g.center.getY());
      }
    }
    agents_initialized_ = true;
    printf("SFM Agents initialized\n");
  }

  void AgentManager::initializeRobot(
      const hunav_msgs::msg::Agent::SharedPtr msg)
  {

    robot_.name = msg->name;
    robot_.gesture = 0;
    robot_.type = msg->type;
    robot_.behavior = msg->behavior;
    robot_.sfmAgent.id = msg->id;
    robot_.sfmAgent.groupId = msg->group_id;
    robot_.sfmAgent.desiredVelocity = msg->desired_velocity;
    robot_.sfmAgent.radius = msg->radius;
    robot_.sfmAgent.cyclicGoals = msg->cyclic_goals;
    robot_.sfmAgent.position.set(msg->position.position.x,
                                 msg->position.position.y);
    robot_.sfmAgent.yaw.setRadian(msg->yaw);
    robot_.sfmAgent.velocity.set(msg->velocity.linear.x, msg->velocity.linear.y);
    robot_.sfmAgent.linearVelocity =
        sqrt(msg->velocity.linear.x * msg->velocity.linear.x +
             msg->velocity.linear.y * msg->velocity.linear.y);
    robot_.sfmAgent.angularVelocity = msg->velocity.angular.z;

    printf("\trobot %i, x:%.2f, y:%.2f\n", robot_.sfmAgent.id,
           robot_.sfmAgent.position.getX(), robot_.sfmAgent.position.getY());

    robot_initialized_ = true;
    printf("SFM Robot initialized\n");
  }

  bool AgentManager::updateAgents(const hunav_msgs::msg::Agents::SharedPtr msg)
  {

    // Update agents velocites only in the ROS cycle
    for (auto a : msg->agents)
    {

      // position
      agents_[a.id].sfmAgent.position.set(a.position.position.x,
                                          a.position.position.y);
      agents_[a.id].sfmAgent.yaw.setRadian(a.yaw);

      // velocities
      agents_[a.id].sfmAgent.velocity.set(a.velocity.linear.x,
                                          a.velocity.linear.y);
      agents_[a.id].sfmAgent.linearVelocity =
          sqrt(a.velocity.linear.x * a.velocity.linear.x +
               a.velocity.linear.y * a.velocity.linear.y);
      agents_[a.id].sfmAgent.angularVelocity = a.velocity.angular.z;
      //std::cout<<agents_[a.id].gesture<<std::endl;

      // update closest obstacles
      agents_[a.id].sfmAgent.obstacles1.clear();
      
      agents_[a.id].gesture = a.gesture;
      agents_[a.id].pause_nav = a.pause_nav;
      if (!a.closest_obs.empty())
      {
        for (auto obs : a.closest_obs)
        {
          utils::Vector2d o;
          o.set(obs.x, obs.y);
          agents_[a.id].sfmAgent.obstacles1.push_back(o);
        }
      }
    }
    step_count = 1;
   
    return true;
  }

  void AgentManager::updateAgentRobot(
      const hunav_msgs::msg::Agent::SharedPtr msg)
  {
    robot_.sfmAgent.position.set(msg->position.position.x,
                                 msg->position.position.y);
    robot_.sfmAgent.yaw.setRadian(msg->yaw);
    robot_.sfmAgent.velocity.set(msg->velocity.linear.x, msg->velocity.linear.y);
    robot_.sfmAgent.linearVelocity =
        sqrt(msg->velocity.linear.x * msg->velocity.linear.x +
             msg->velocity.linear.y * msg->velocity.linear.y);
    robot_.sfmAgent.angularVelocity = msg->velocity.angular.z;
    robot_.gesture = msg->gesture;
  }

  hunav_msgs::msg::Agent AgentManager::getUpdatedAgentMsg(int id)
  {
    // std::lock_guard<std::mutex> guard(mutex_);
    hunav_msgs::msg::Agent a;
    a.id = agents_[id].sfmAgent.id;
    a.name = agents_[id].name;
    a.type = agents_[id].type;
    a.behavior = agents_[id].behavior;
    a.behavior_state = agents_[id].behavior_state;
    
    //a.current_goal_index = agents_[id].current_goal_index;
    
    a.position.position.x = agents_[id].sfmAgent.position.getX();
    a.position.position.y = agents_[id].sfmAgent.position.getY();
    a.yaw = agents_[id].sfmAgent.yaw.toRadian();
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, agents_[id].sfmAgent.yaw.toRadian());
    a.position.orientation = tf2::toMsg(myQuaternion);
    a.linear_vel = agents_[id].sfmAgent.linearVelocity;
    a.angular_vel = agents_[id].sfmAgent.angularVelocity;
    a.velocity.linear.x = agents_[id].sfmAgent.velocity.getX();
    a.velocity.linear.y = agents_[id].sfmAgent.velocity.getY();
    a.velocity.angular.z = agents_[id].sfmAgent.angularVelocity;
    a.gesture = agents_[id].gesture;
    for (auto g : agents_[id].sfmAgent.goals)
    {
      geometry_msgs::msg::Pose p;
      p.position.x = g.center.getX();
      p.position.y = g.center.getY();
      a.goals.push_back(p);
    }
    return a;
  }

  // create a agents msg from the sfm_agents_
  hunav_msgs::msg::Agents AgentManager::getUpdatedAgentsMsg()
  {
    // std::cout<<"UPDATING AGENTS MSG\n"<<std::endl<<std::endl;
    // Make a copy of agents_
    // Then update them with the sfm_agents_ data
    std::lock_guard<std::mutex> guard(mutex_);
    hunav_msgs::msg::Agents agents_msg;
    agents_msg.header = header_;

    std::unordered_map<int, agent>::iterator itr;
    for (itr = agents_.begin(); itr != agents_.end(); itr++)
    {
      //std::cout<<"Current Gestures:"<<agents_[itr->first].gesture<<std::endl;
      hunav_msgs::msg::Agent a = getUpdatedAgentMsg(itr->first);
      agents_msg.agents.push_back(a);
    }
    robot_received_ = false;
    agents_received_ = false;
    return agents_msg;
  }

  std::vector<sfm::Agent> AgentManager::getSFMAgents()
  {

    std::vector<sfm::Agent> agents;
    std::unordered_map<int, agent>::iterator itr;
    for (itr = agents_.begin(); itr != agents_.end(); itr++)
    {
      agents.push_back(itr->second.sfmAgent);
    }
    return agents;
  }

  void AgentManager::computeForces(int id)
  {

    std::vector<sfm::Agent> otherAgents = getSFMAgents();

    utils::Vector2d ob;
    switch (agents_[id].behavior)
    {
    case hunav_msgs::msg::Agent::BEH_REGULAR:
      // We add the robot as another human agent.
      otherAgents.push_back(robot_.sfmAgent);
      sfm::SFM.computeForces(agents_[id].sfmAgent, otherAgents);
      break;
    case hunav_msgs::msg::Agent::BEH_IMPASSIVE:
      // the human treats the robot like an obstacle.
      // We add the robot to the obstacles of this agent.
      ob.set(robot_.sfmAgent.position.getX(), robot_.sfmAgent.position.getY());
      agents_[id].sfmAgent.obstacles1.push_back(ob);
      sfm::SFM.computeForces(agents_[id].sfmAgent, otherAgents);
      break;
    default:
      // Compute forces as usual (not taking into account the robot)
      sfm::SFM.computeForces(agents_[id].sfmAgent, otherAgents);
    }

    bool compute = false;
    if (isnan(agents_[id].sfmAgent.forces.desiredForce.norm()))
    {
      printf("[AgentManager.ComputeForces] \tgoal force is nan. Using zero.. \n");
      agents_[id].sfmAgent.forces.desiredForce.set(0, 0);
      compute = true;
    }
    if (isnan(agents_[id].sfmAgent.forces.groupForce.norm()))
    {
      printf("[AgentManager.ComputeForces] \tgroup force is nan. Using zero.. "
             "\n");
      agents_[id].sfmAgent.forces.groupForce.set(0, 0);
      compute = true;
    }
    if (isnan(agents_[id].sfmAgent.forces.obstacleForce.norm()))
    {
      printf("[AgentManager.ComputeForces] \tobstacle force is nan. Using "
             "zero.. \n");
      agents_[id].sfmAgent.forces.obstacleForce.set(0, 0);
      compute = true;
    }
    if (isnan(agents_[id].sfmAgent.forces.socialForce.norm()))
    {
      printf("[AgentManager.ComputeForces] \tsocial force is nan. Using "
             "zero.. \n");
      agents_[id].sfmAgent.forces.socialForce.set(0, 0);
      compute = true;
    }
    if (compute)
    {
      agents_[id].sfmAgent.forces.globalForce =
          agents_[id].sfmAgent.forces.desiredForce +
          agents_[id].sfmAgent.forces.socialForce +
          agents_[id].sfmAgent.forces.obstacleForce +
          agents_[id].sfmAgent.forces.groupForce;
    }
  }

  void AgentManager::computeForces()
  {

    std::vector<sfm::Agent> otherAgents = getSFMAgents();

    std::unordered_map<int, agent>::iterator itr;
    for (itr = agents_.begin(); itr != agents_.end(); itr++)
    {
      computeForces(itr->second.sfmAgent.id);
    }
  }

  void AgentManager::updateAllAgents(
      const hunav_msgs::msg::Agent::SharedPtr robot_msg,
      const hunav_msgs::msg::Agents::SharedPtr agents_msg)
  {

    std::lock_guard<std::mutex> guard(mutex_);
    header_ = agents_msg->header;

    if (!robot_initialized_)
    {
      initializeRobot(robot_msg);
    }

    if (!agents_initialized_)
    {
      initializeAgents(agents_msg);
    }
    else if (robot_initialized_ && agents_initialized_)
    {
      updateAgentRobot(robot_msg);
      move = updateAgents(agents_msg);
    }
    
    agents_received_ = true;
    robot_received_ = true;
    computeForces();
  }

  void AgentManager::updateAgentsAndRobot(
      const hunav_msgs::msg::Agents::SharedPtr agents_msg)
  {

    std::lock_guard<std::mutex> guard(mutex_);

    header_ = agents_msg->header;

    // The robot is the last agent of the vector!
    // or we could look for the type "robot" in the vector
    hunav_msgs::msg::Agent::SharedPtr rob =
        std::make_shared<hunav_msgs::msg::Agent>(agents_msg->agents.back());

    // we remove the robot from the agents vector
    hunav_msgs::msg::Agents ags = *agents_msg;
    ags.agents.pop_back();

    if (!robot_initialized_)
    {
      initializeRobot(rob);
    }
    if (!agents_initialized_)
    {
      initializeAgents(std::make_shared<hunav_msgs::msg::Agents>(ags));
    }
    else
    {
      updateAgentRobot(rob);
      move = updateAgents(std::make_shared<hunav_msgs::msg::Agents>(ags));
    }

    agents_received_ = true;
    robot_received_ = true;
    computeForces();
  }

  bool AgentManager::canCompute()
  {
    if (agents_received_ && robot_received_)
      return true;
    else
      return false;
  }
}