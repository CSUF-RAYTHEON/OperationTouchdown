#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__srv__TriggerNamed_Request() -> *const std::ffi::c_void;
}

#[link(name = "depthai_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn depthai_ros_msgs__srv__TriggerNamed_Request__init(msg: *mut TriggerNamed_Request) -> bool;
    fn depthai_ros_msgs__srv__TriggerNamed_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TriggerNamed_Request>, size: usize) -> bool;
    fn depthai_ros_msgs__srv__TriggerNamed_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TriggerNamed_Request>);
    fn depthai_ros_msgs__srv__TriggerNamed_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TriggerNamed_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<TriggerNamed_Request>) -> bool;
}

// Corresponds to depthai_ros_msgs__srv__TriggerNamed_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TriggerNamed_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub name: rosidl_runtime_rs::String,

}



impl Default for TriggerNamed_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !depthai_ros_msgs__srv__TriggerNamed_Request__init(&mut msg as *mut _) {
        panic!("Call to depthai_ros_msgs__srv__TriggerNamed_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TriggerNamed_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__srv__TriggerNamed_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__srv__TriggerNamed_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__srv__TriggerNamed_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TriggerNamed_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TriggerNamed_Request where Self: Sized {
  const TYPE_NAME: &'static str = "depthai_ros_msgs/srv/TriggerNamed_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__srv__TriggerNamed_Request() }
  }
}


#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__srv__TriggerNamed_Response() -> *const std::ffi::c_void;
}

#[link(name = "depthai_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn depthai_ros_msgs__srv__TriggerNamed_Response__init(msg: *mut TriggerNamed_Response) -> bool;
    fn depthai_ros_msgs__srv__TriggerNamed_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TriggerNamed_Response>, size: usize) -> bool;
    fn depthai_ros_msgs__srv__TriggerNamed_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TriggerNamed_Response>);
    fn depthai_ros_msgs__srv__TriggerNamed_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TriggerNamed_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<TriggerNamed_Response>) -> bool;
}

// Corresponds to depthai_ros_msgs__srv__TriggerNamed_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TriggerNamed_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for TriggerNamed_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !depthai_ros_msgs__srv__TriggerNamed_Response__init(&mut msg as *mut _) {
        panic!("Call to depthai_ros_msgs__srv__TriggerNamed_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TriggerNamed_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__srv__TriggerNamed_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__srv__TriggerNamed_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__srv__TriggerNamed_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TriggerNamed_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TriggerNamed_Response where Self: Sized {
  const TYPE_NAME: &'static str = "depthai_ros_msgs/srv/TriggerNamed_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__srv__TriggerNamed_Response() }
  }
}


#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__srv__NormalizedImageCrop_Request() -> *const std::ffi::c_void;
}

#[link(name = "depthai_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn depthai_ros_msgs__srv__NormalizedImageCrop_Request__init(msg: *mut NormalizedImageCrop_Request) -> bool;
    fn depthai_ros_msgs__srv__NormalizedImageCrop_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NormalizedImageCrop_Request>, size: usize) -> bool;
    fn depthai_ros_msgs__srv__NormalizedImageCrop_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NormalizedImageCrop_Request>);
    fn depthai_ros_msgs__srv__NormalizedImageCrop_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NormalizedImageCrop_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<NormalizedImageCrop_Request>) -> bool;
}

// Corresponds to depthai_ros_msgs__srv__NormalizedImageCrop_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NormalizedImageCrop_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub top_left: geometry_msgs::msg::rmw::Pose2D,


    // This member is not documented.
    #[allow(missing_docs)]
    pub bottom_right: geometry_msgs::msg::rmw::Pose2D,

}



impl Default for NormalizedImageCrop_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !depthai_ros_msgs__srv__NormalizedImageCrop_Request__init(&mut msg as *mut _) {
        panic!("Call to depthai_ros_msgs__srv__NormalizedImageCrop_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NormalizedImageCrop_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__srv__NormalizedImageCrop_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__srv__NormalizedImageCrop_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__srv__NormalizedImageCrop_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NormalizedImageCrop_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NormalizedImageCrop_Request where Self: Sized {
  const TYPE_NAME: &'static str = "depthai_ros_msgs/srv/NormalizedImageCrop_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__srv__NormalizedImageCrop_Request() }
  }
}


#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__srv__NormalizedImageCrop_Response() -> *const std::ffi::c_void;
}

#[link(name = "depthai_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn depthai_ros_msgs__srv__NormalizedImageCrop_Response__init(msg: *mut NormalizedImageCrop_Response) -> bool;
    fn depthai_ros_msgs__srv__NormalizedImageCrop_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NormalizedImageCrop_Response>, size: usize) -> bool;
    fn depthai_ros_msgs__srv__NormalizedImageCrop_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NormalizedImageCrop_Response>);
    fn depthai_ros_msgs__srv__NormalizedImageCrop_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NormalizedImageCrop_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<NormalizedImageCrop_Response>) -> bool;
}

// Corresponds to depthai_ros_msgs__srv__NormalizedImageCrop_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NormalizedImageCrop_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i64,

}



impl Default for NormalizedImageCrop_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !depthai_ros_msgs__srv__NormalizedImageCrop_Response__init(&mut msg as *mut _) {
        panic!("Call to depthai_ros_msgs__srv__NormalizedImageCrop_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NormalizedImageCrop_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__srv__NormalizedImageCrop_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__srv__NormalizedImageCrop_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__srv__NormalizedImageCrop_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NormalizedImageCrop_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NormalizedImageCrop_Response where Self: Sized {
  const TYPE_NAME: &'static str = "depthai_ros_msgs/srv/NormalizedImageCrop_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__srv__NormalizedImageCrop_Response() }
  }
}






#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__depthai_ros_msgs__srv__TriggerNamed() -> *const std::ffi::c_void;
}

// Corresponds to depthai_ros_msgs__srv__TriggerNamed
#[allow(missing_docs, non_camel_case_types)]
pub struct TriggerNamed;

impl rosidl_runtime_rs::Service for TriggerNamed {
    type Request = TriggerNamed_Request;
    type Response = TriggerNamed_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__depthai_ros_msgs__srv__TriggerNamed() }
    }
}




#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__depthai_ros_msgs__srv__NormalizedImageCrop() -> *const std::ffi::c_void;
}

// Corresponds to depthai_ros_msgs__srv__NormalizedImageCrop
#[allow(missing_docs, non_camel_case_types)]
pub struct NormalizedImageCrop;

impl rosidl_runtime_rs::Service for NormalizedImageCrop {
    type Request = NormalizedImageCrop_Request;
    type Response = NormalizedImageCrop_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__depthai_ros_msgs__srv__NormalizedImageCrop() }
    }
}


