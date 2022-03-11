#ifndef _GEOMETRY_MSGS_MSG_POSE_H
#define _GEOMETRY_MSGS_MSG_POSE_H

#include <iostream>
#include <string>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace std;

namespace geometry_msgs
{
namespace msg
{
class Pose
{
public:
  uint32_t cntPub = 0;
  uint32_t cntSub = 0;

    
  geometry_msgs::msg::Point position
;
    
  geometry_msgs::msg::Quaternion orientation;
  

  uint32_t copyToBuf(uint8_t *addrPtr)
  {
    uint32_t tmpPub = 0;
    uint32_t arraySize;
    uint32_t stringSize;
    
    
    
    tmpPub = position
.copyToBuf(addrPtr);
    cntPub += tmpPub;
    addrPtr += tmpPub;
    
    
    
    
    
    tmpPub = orientation.copyToBuf(addrPtr);
    cntPub += tmpPub;
    addrPtr += tmpPub;
    
    
    

    return cntPub;
  }

  uint32_t copyFromBuf(const uint8_t *addrPtr) {
    uint32_t tmpSub = 0;
    uint32_t arraySize;
    uint32_t stringSize;

    
    
    
    tmpSub = position
.copyFromBuf(addrPtr);
    cntSub += tmpSub;
    addrPtr += tmpSub;
    

    
    
    
    
    tmpSub = orientation.copyFromBuf(addrPtr);
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
  std::string type_name = "geometry_msgs::msg::dds_::Pose";
};
};
}

namespace message_traits
{
template<>
struct TypeName<geometry_msgs::msg::Pose*> {
  static const char* value()
  {
    return "geometry_msgs::msg::dds_::Pose_";
  }
};
}

#endif