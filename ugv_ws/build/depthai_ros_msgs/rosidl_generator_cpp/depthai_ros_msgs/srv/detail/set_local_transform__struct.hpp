// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from depthai_ros_msgs:srv/SetLocalTransform.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "depthai_ros_msgs/srv/set_local_transform.hpp"


#ifndef DEPTHAI_ROS_MSGS__SRV__DETAIL__SET_LOCAL_TRANSFORM__STRUCT_HPP_
#define DEPTHAI_ROS_MSGS__SRV__DETAIL__SET_LOCAL_TRANSFORM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'local_transform'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__depthai_ros_msgs__srv__SetLocalTransform_Request __attribute__((deprecated))
#else
# define DEPRECATED__depthai_ros_msgs__srv__SetLocalTransform_Request __declspec(deprecated)
#endif

namespace depthai_ros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLocalTransform_Request_
{
  using Type = SetLocalTransform_Request_<ContainerAllocator>;

  explicit SetLocalTransform_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : local_transform(_init)
  {
    (void)_init;
  }

  explicit SetLocalTransform_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : local_transform(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _local_transform_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _local_transform_type local_transform;

  // setters for named parameter idiom
  Type & set__local_transform(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->local_transform = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__depthai_ros_msgs__srv__SetLocalTransform_Request
    std::shared_ptr<depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__depthai_ros_msgs__srv__SetLocalTransform_Request
    std::shared_ptr<depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLocalTransform_Request_ & other) const
  {
    if (this->local_transform != other.local_transform) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLocalTransform_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLocalTransform_Request_

// alias to use template instance with default allocator
using SetLocalTransform_Request =
  depthai_ros_msgs::srv::SetLocalTransform_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace depthai_ros_msgs


#ifndef _WIN32
# define DEPRECATED__depthai_ros_msgs__srv__SetLocalTransform_Response __attribute__((deprecated))
#else
# define DEPRECATED__depthai_ros_msgs__srv__SetLocalTransform_Response __declspec(deprecated)
#endif

namespace depthai_ros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLocalTransform_Response_
{
  using Type = SetLocalTransform_Response_<ContainerAllocator>;

  explicit SetLocalTransform_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetLocalTransform_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__depthai_ros_msgs__srv__SetLocalTransform_Response
    std::shared_ptr<depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__depthai_ros_msgs__srv__SetLocalTransform_Response
    std::shared_ptr<depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLocalTransform_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLocalTransform_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLocalTransform_Response_

// alias to use template instance with default allocator
using SetLocalTransform_Response =
  depthai_ros_msgs::srv::SetLocalTransform_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace depthai_ros_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__depthai_ros_msgs__srv__SetLocalTransform_Event __attribute__((deprecated))
#else
# define DEPRECATED__depthai_ros_msgs__srv__SetLocalTransform_Event __declspec(deprecated)
#endif

namespace depthai_ros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLocalTransform_Event_
{
  using Type = SetLocalTransform_Event_<ContainerAllocator>;

  explicit SetLocalTransform_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit SetLocalTransform_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<depthai_ros_msgs::srv::SetLocalTransform_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<depthai_ros_msgs::srv::SetLocalTransform_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    depthai_ros_msgs::srv::SetLocalTransform_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const depthai_ros_msgs::srv::SetLocalTransform_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<depthai_ros_msgs::srv::SetLocalTransform_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<depthai_ros_msgs::srv::SetLocalTransform_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      depthai_ros_msgs::srv::SetLocalTransform_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<depthai_ros_msgs::srv::SetLocalTransform_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      depthai_ros_msgs::srv::SetLocalTransform_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<depthai_ros_msgs::srv::SetLocalTransform_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<depthai_ros_msgs::srv::SetLocalTransform_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<depthai_ros_msgs::srv::SetLocalTransform_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__depthai_ros_msgs__srv__SetLocalTransform_Event
    std::shared_ptr<depthai_ros_msgs::srv::SetLocalTransform_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__depthai_ros_msgs__srv__SetLocalTransform_Event
    std::shared_ptr<depthai_ros_msgs::srv::SetLocalTransform_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLocalTransform_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLocalTransform_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLocalTransform_Event_

// alias to use template instance with default allocator
using SetLocalTransform_Event =
  depthai_ros_msgs::srv::SetLocalTransform_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace depthai_ros_msgs

namespace depthai_ros_msgs
{

namespace srv
{

struct SetLocalTransform
{
  using Request = depthai_ros_msgs::srv::SetLocalTransform_Request;
  using Response = depthai_ros_msgs::srv::SetLocalTransform_Response;
  using Event = depthai_ros_msgs::srv::SetLocalTransform_Event;
};

}  // namespace srv

}  // namespace depthai_ros_msgs

#endif  // DEPTHAI_ROS_MSGS__SRV__DETAIL__SET_LOCAL_TRANSFORM__STRUCT_HPP_
