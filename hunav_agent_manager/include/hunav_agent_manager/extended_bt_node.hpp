#ifndef HUNAV_BEHAVIORS__EXTENDED_BT_NODE_HPP_
#define HUNAV_BEHAVIORS__EXTENDED_BT_NODE_HPP_
#include "bt_node.hpp"

namespace hunav{

class BTnodeExt : public BTnode {
public:
    BTnodeExt();
    virtual ~BTnodeExt();

    //<NEW PUBLIC MEMBER>
    
    
    //<NEW PUBLIC FUNCTION>

    void registerBTNodes();
private:
    //<NEW PRIVATE MEMBER>
    

    //<NEW PRIVATE FUNCTION>
};
}
#endif