// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rtcm_msgs:msg/Message.idl
// generated code does not contain a copyright notice

#ifndef RTCM_MSGS__MSG__DETAIL__MESSAGE__BUILDER_HPP_
#define RTCM_MSGS__MSG__DETAIL__MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rtcm_msgs/msg/detail/message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rtcm_msgs
{

namespace msg
{

namespace builder
{

class Init_Message_message
{
public:
  explicit Init_Message_message(::rtcm_msgs::msg::Message & msg)
  : msg_(msg)
  {}
  ::rtcm_msgs::msg::Message message(::rtcm_msgs::msg::Message::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rtcm_msgs::msg::Message msg_;
};

class Init_Message_header
{
public:
  Init_Message_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Message_message header(::rtcm_msgs::msg::Message::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Message_message(msg_);
  }

private:
  ::rtcm_msgs::msg::Message msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rtcm_msgs::msg::Message>()
{
  return rtcm_msgs::msg::builder::Init_Message_header();
}

}  // namespace rtcm_msgs

#endif  // RTCM_MSGS__MSG__DETAIL__MESSAGE__BUILDER_HPP_
