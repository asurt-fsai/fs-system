// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lidarslam_msgs:msg/SubMap.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lidarslam_msgs/msg/detail/sub_map__rosidl_typesupport_introspection_c.h"
#include "lidarslam_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lidarslam_msgs/msg/detail/sub_map__functions.h"
#include "lidarslam_msgs/msg/detail/sub_map__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `cloud`
#include "sensor_msgs/msg/point_cloud2.h"
// Member `cloud`
#include "sensor_msgs/msg/detail/point_cloud2__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lidarslam_msgs__msg__SubMap__init(message_memory);
}

void lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_fini_function(void * message_memory)
{
  lidarslam_msgs__msg__SubMap__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidarslam_msgs__msg__SubMap, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidarslam_msgs__msg__SubMap, distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidarslam_msgs__msg__SubMap, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cloud",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidarslam_msgs__msg__SubMap, cloud),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_message_members = {
  "lidarslam_msgs__msg",  // message namespace
  "SubMap",  // message name
  4,  // number of fields
  sizeof(lidarslam_msgs__msg__SubMap),
  lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_message_member_array,  // message members
  lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_init_function,  // function to initialize message memory (memory has to be allocated)
  lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_message_type_support_handle = {
  0,
  &lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lidarslam_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidarslam_msgs, msg, SubMap)() {
  lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  if (!lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_message_type_support_handle.typesupport_identifier) {
    lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &lidarslam_msgs__msg__SubMap__rosidl_typesupport_introspection_c__SubMap_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
