
#include "geometry_msgs/msg/pose.hpp"




template mros2::Subscriber mros2::Node::create_subscription(std::string topic_name, int qos, void (*fp)(geometry_msgs::msg::Pose*));
template void mros2::Subscriber::callback_handler<geometry_msgs::msg::Pose>(void *callee, const rtps::ReaderCacheChange &cacheChange);
