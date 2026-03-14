// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rtcm_msgs:msg/Message.idl
// generated code does not contain a copyright notice

#ifndef RTCM_MSGS__MSG__DETAIL__MESSAGE__TRAITS_HPP_
#define RTCM_MSGS__MSG__DETAIL__MESSAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rtcm_msgs/msg/detail/message__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rtcm_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Message & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: message
  {
    if (msg.message.size() == 0) {
      out << "message: []";
    } else {
      out << "message: [";
      size_t pending_items = msg.message.size();
      for (auto item : msg.message) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Message & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.message.size() == 0) {
      out << "message: []\n";
    } else {
      out << "message:\n";
      for (auto item : msg.message) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Message & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rtcm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rtcm_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rtcm_msgs::msg::Message & msg,
  std::ostream & out, size_t indentation = 0)
{
  rtcm_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rtcm_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rtcm_msgs::msg::Message & msg)
{
  return rtcm_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rtcm_msgs::msg::Message>()
{
  return "rtcm_msgs::msg::Message";
}

template<>
inline const char * name<rtcm_msgs::msg::Message>()
{
  return "rtcm_msgs/msg/Message";
}

template<>
struct has_fixed_size<rtcm_msgs::msg::Message>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rtcm_msgs::msg::Message>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rtcm_msgs::msg::Message>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RTCM_MSGS__MSG__DETAIL__MESSAGE__TRAITS_HPP_
