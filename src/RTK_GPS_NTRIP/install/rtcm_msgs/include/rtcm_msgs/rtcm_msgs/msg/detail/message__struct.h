// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rtcm_msgs:msg/Message.idl
// generated code does not contain a copyright notice

#ifndef RTCM_MSGS__MSG__DETAIL__MESSAGE__STRUCT_H_
#define RTCM_MSGS__MSG__DETAIL__MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'message'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Message in the package rtcm_msgs.
/**
  * A message representing a single RTCM message.
 */
typedef struct rtcm_msgs__msg__Message
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__uint8__Sequence message;
} rtcm_msgs__msg__Message;

// Struct for a sequence of rtcm_msgs__msg__Message.
typedef struct rtcm_msgs__msg__Message__Sequence
{
  rtcm_msgs__msg__Message * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rtcm_msgs__msg__Message__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RTCM_MSGS__MSG__DETAIL__MESSAGE__STRUCT_H_
