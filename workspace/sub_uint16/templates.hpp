
#include "std_msgs/msg/u_int16.hpp"




template mros2::Subscriber mros2::Node::create_subscription(std::string topic_name, int qos, void (*fp)(std_msgs::msg::UInt16*));
template void mros2::Subscriber::callback_handler<std_msgs::msg::UInt16>(void *callee, const rtps::ReaderCacheChange &cacheChange);
