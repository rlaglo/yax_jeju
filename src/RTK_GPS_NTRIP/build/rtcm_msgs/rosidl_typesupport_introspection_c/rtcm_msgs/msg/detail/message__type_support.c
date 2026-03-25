// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rtcm_msgs:msg/Message.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rtcm_msgs/msg/detail/message__rosidl_typesupport_introspection_c.h"
#include "rtcm_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rtcm_msgs/msg/detail/message__functions.h"
#include "rtcm_msgs/msg/detail/message__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `message`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__Message_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rtcm_msgs__msg__Message__init(message_memory);
}

void rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__Message_fini_function(void * message_memory)
{
  rtcm_msgs__msg__Message__fini(message_memory);
}

size_t rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__size_function__Message__message(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__get_const_function__Message__message(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__get_function__Message__message(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__fetch_function__Message__message(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__get_const_function__Message__message(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__assign_function__Message__message(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__get_function__Message__message(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__resize_function__Message__message(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__Message_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rtcm_msgs__msg__Message, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rtcm_msgs__msg__Message, message),  // bytes offset in struct
    NULL,  // default value
    rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__size_function__Message__message,  // size() function pointer
    rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__get_const_function__Message__message,  // get_const(index) function pointer
    rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__get_function__Message__message,  // get(index) function pointer
    rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__fetch_function__Message__message,  // fetch(index, &value) function pointer
    rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__assign_function__Message__message,  // assign(index, value) function pointer
    rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__resize_function__Message__message  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__Message_message_members = {
  "rtcm_msgs__msg",  // message namespace
  "Message",  // message name
  2,  // number of fields
  sizeof(rtcm_msgs__msg__Message),
  rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__Message_message_member_array,  // message members
  rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__Message_init_function,  // function to initialize message memory (memory has to be allocated)
  rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__Message_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__Message_message_type_support_handle = {
  0,
  &rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__Message_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rtcm_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rtcm_msgs, msg, Message)() {
  rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__Message_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__Message_message_type_support_handle.typesupport_identifier) {
    rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__Message_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rtcm_msgs__msg__Message__rosidl_typesupport_introspection_c__Message_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
