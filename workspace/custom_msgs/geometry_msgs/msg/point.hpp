#ifndef _GEOMETRY_MSGS_MSG_POINT_H
#define _GEOMETRY_MSGS_MSG_POINT_H

#include <iostream>
#include <string>

using namespace std;

namespace geometry_msgs
{
namespace msg
{
class Point
{
public:
  uint32_t cntPub = 0;
  uint32_t cntSub = 0;

    
  double x
;
    
  double y
;
    
  double z;
  

  uint32_t copyToBuf(uint8_t *addrPtr)
  {
    uint32_t tmpPub = 0;
    uint32_t arraySize;
    uint32_t stringSize;
    
    
    
    if (cntPub%8 > 0){
      for(int i=0; i<(8-(cntPub%8)) ; i++){
        *addrPtr = 0;
        addrPtr += 1;
      }   
      cntPub += 8-(cntPub%8);
    }
    
    memcpy(addrPtr,&x
,8);
    addrPtr += 8;
    cntPub += 8;

    
    
    
    
    if (cntPub%8 > 0){
      for(int i=0; i<(8-(cntPub%8)) ; i++){
        *addrPtr = 0;
        addrPtr += 1;
      }   
      cntPub += 8-(cntPub%8);
    }
    
    memcpy(addrPtr,&y
,8);
    addrPtr += 8;
    cntPub += 8;

    
    
    
    
    if (cntPub%8 > 0){
      for(int i=0; i<(8-(cntPub%8)) ; i++){
        *addrPtr = 0;
        addrPtr += 1;
      }   
      cntPub += 8-(cntPub%8);
    }
    
    memcpy(addrPtr,&z,8);
    addrPtr += 8;
    cntPub += 8;

    
    

    return cntPub;
  }

  uint32_t copyFromBuf(const uint8_t *addrPtr) {
    uint32_t tmpSub = 0;
    uint32_t arraySize;
    uint32_t stringSize;

    
    
    
    if (cntSub%8 > 0){
      for(int i=0; i<(8-(cntSub%8)) ; i++){
        addrPtr += 1;
      }   
      cntSub += 8-(cntSub%8);
    }
    
    memcpy(&x
,addrPtr,8);
    addrPtr += 8;
    cntSub += 8;
    
    
    
    
    if (cntSub%8 > 0){
      for(int i=0; i<(8-(cntSub%8)) ; i++){
        addrPtr += 1;
      }   
      cntSub += 8-(cntSub%8);
    }
    
    memcpy(&y
,addrPtr,8);
    addrPtr += 8;
    cntSub += 8;
    
    
    
    
    if (cntSub%8 > 0){
      for(int i=0; i<(8-(cntSub%8)) ; i++){
        addrPtr += 1;
      }   
      cntSub += 8-(cntSub%8);
    }
    
    memcpy(&z,addrPtr,8);
    addrPtr += 8;
    cntSub += 8;
    
    

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
  std::string type_name = "geometry_msgs::msg::dds_::Point";
};
};
}

namespace message_traits
{
template<>
struct TypeName<geometry_msgs::msg::Point*> {
  static const char* value()
  {
    return "geometry_msgs::msg::dds_::Point_";
  }
};
}

#endif