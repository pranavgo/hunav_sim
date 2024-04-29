#ifndef HUNAV_BEHAVIORS__EXTENDED_BT_FUNCTIONS_HPP_
#define HUNAV_BEHAVIORS__EXTENDED_BT_FUNCTIONS_HPP_
#include "bt_functions.hpp"

namespace hunav{
class BTfunctionsExt : public BTfunctions {
public:
    BTfunctionsExt();
    virtual ~BTfunctionsExt();

    //<NEW PUBLIC MEMBER>
    
    
    //<NEW PUBLIC FUNCTION> 
BT::NodeStatus heardProceed(BT::TreeNode &self); 
    
private:
    //<NEW PRIVATE MEMBER>
    

    //<NEW PRIVATE FUNCTION>
};
}
#endif