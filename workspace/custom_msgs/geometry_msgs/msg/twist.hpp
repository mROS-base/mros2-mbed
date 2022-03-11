#ifndef _GEOMETRY_MSGS_MSG_TWIST_H
#define _GEOMETRY_MSGS_MSG_TWIST_H

#include <iostream>
#include <string>
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std;

namespace geometry_msgs
{
namespace msg
{
class Twist
{
public:
  uint32_t cntPub = 0;
  uint32_t cntSub = 0;

    
  geometry_msgs::msg::Vector3 linear
;
    
  geometry_msgs::msg::Vector3 angular;
  

  uint32_t copyToBuf(uint8_t *addrPtr)
  {
    uint32_t tmpPub = 0;
    uint32_t arraySize;
    uint32_t stringSize;
    
    
    
    tmpPub = linear
.copyToBuf(addrPtr);
    cntPub += tmpPub;
    addrPtr += tmpPub;
    
    
    
    
    
    tmpPub = angular.copyToBuf(addrPtr);
    cntPub += tmpPub;
    addrPtr += tmpPub;
    
    
    

    return cntPub;
  }

  uint32_t copyFromBuf(const uint8_t *addrPtr) {
    uint32_t tmpSub = 0;
    uint32_t arraySize;
    uint32_t stringSize;

    
    
    
    tmpSub = linear
.copyFromBuf(addrPtr);
    cntSub += tmpSub;
    addrPtr += tmpSub;
    

    
    
    
    
    tmpSub = angular.copyFromBuf(addrPtr);
    cntSub += tmpSub;
    addrPtr += tmpSub;
    

    
    

    return cntSub;
  }

   void memAlign(uint8_t *addrPtr){
    if (cntPub%4 > 0){
      addrPtr += cntPub;
      for(int i=0; i<(4-(cntPub%4)) ; i++){
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4-(cntPub%4);
    }
    return;
  }

  uint32_t getTotalSize(){
    uint32_t tmpCntPub = cntPub;
    cntPub = 0;
    return tmpCntPub ;
  }

private:
  std::string type_name = "geometry_msgs::msg::dds_::Twist";
};
};
}

namespace message_traits
{
template<>
struct TypeName<geometry_msgs::msg::Twist*> {
  static const char* value()
  {
    return "geometry_msgs::msg::dds_::Twist_";
  }
};
}

#endif