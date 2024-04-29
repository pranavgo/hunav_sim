#include "hunav_agent_manager/extended_bt_node.hpp"

namespace hunav {

using std::placeholders::_1;
using std::placeholders::_2;
BTnodeExt::BTnodeExt() {
    // Initialization of new variables or functionalities
      registerBTNodes();
}

BTnodeExt::~BTnodeExt() {
    // Cleanup code if necessary

}

void BTnodeExt::registerBTNodes()
  {
    BT::PortsList simple_port = {BT::InputPort<int>("agent_id")}; 
    BT::PortsList visibleports = {BT::InputPort<int>("agent_id"),
                                  BT::InputPort<double>("distance")};
    BT::PortsList portsNav = {BT::InputPort<int>("agent_id"),
                              BT::InputPort<double>("time_step")};
    
    BT::PortsList portsMsg = {BT::InputPort<int>("agent_id"),
                              BT::InputPort<int>("message")};  
                              
    //<NEW NODE REGISTER>   
    RCLCPP_INFO(this->get_logger(), "BT nodes registered");
  }
//<NEW FUNCTION> 

} // namespace hunav
