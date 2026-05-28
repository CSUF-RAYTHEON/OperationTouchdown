#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__AutoFocusCtrl() -> *const std::ffi::c_void;
}

#[link(name = "depthai_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn depthai_ros_msgs__msg__AutoFocusCtrl__init(msg: *mut AutoFocusCtrl) -> bool;
    fn depthai_ros_msgs__msg__AutoFocusCtrl__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AutoFocusCtrl>, size: usize) -> bool;
    fn depthai_ros_msgs__msg__AutoFocusCtrl__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AutoFocusCtrl>);
    fn depthai_ros_msgs__msg__AutoFocusCtrl__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AutoFocusCtrl>, out_seq: *mut rosidl_runtime_rs::Sequence<AutoFocusCtrl>) -> bool;
}

// Corresponds to depthai_ros_msgs__msg__AutoFocusCtrl
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AutoFocusCtrl {

    // This member is not documented.
    #[allow(missing_docs)]
    pub auto_focus_mode: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub trigger_auto_focus: bool,

}

impl AutoFocusCtrl {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AF_MODE_AUTO: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AF_MODE_MACRO: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AF_MODE_CONTINUOUS_VIDEO: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AF_MODE_CONTINUOUS_PICTURE: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const AF_MODE_EDOF: u8 = 4;

}


impl Default for AutoFocusCtrl {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !depthai_ros_msgs__msg__AutoFocusCtrl__init(&mut msg as *mut _) {
        panic!("Call to depthai_ros_msgs__msg__AutoFocusCtrl__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AutoFocusCtrl {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__AutoFocusCtrl__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__AutoFocusCtrl__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__AutoFocusCtrl__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AutoFocusCtrl {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AutoFocusCtrl where Self: Sized {
  const TYPE_NAME: &'static str = "depthai_ros_msgs/msg/AutoFocusCtrl";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__AutoFocusCtrl() }
  }
}


#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__HandLandmark() -> *const std::ffi::c_void;
}

#[link(name = "depthai_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn depthai_ros_msgs__msg__HandLandmark__init(msg: *mut HandLandmark) -> bool;
    fn depthai_ros_msgs__msg__HandLandmark__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<HandLandmark>, size: usize) -> bool;
    fn depthai_ros_msgs__msg__HandLandmark__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<HandLandmark>);
    fn depthai_ros_msgs__msg__HandLandmark__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<HandLandmark>, out_seq: *mut rosidl_runtime_rs::Sequence<HandLandmark>) -> bool;
}

// Corresponds to depthai_ros_msgs__msg__HandLandmark
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct HandLandmark {
    /// Center of the
    pub label: rosidl_runtime_rs::String,

    /// Landmarks score.
    pub lm_score: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub landmark: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Pose2D>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::rmw::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub is_spatial: bool,

}



impl Default for HandLandmark {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !depthai_ros_msgs__msg__HandLandmark__init(&mut msg as *mut _) {
        panic!("Call to depthai_ros_msgs__msg__HandLandmark__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for HandLandmark {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__HandLandmark__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__HandLandmark__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__HandLandmark__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for HandLandmark {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for HandLandmark where Self: Sized {
  const TYPE_NAME: &'static str = "depthai_ros_msgs/msg/HandLandmark";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__HandLandmark() }
  }
}


#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__HandLandmarkArray() -> *const std::ffi::c_void;
}

#[link(name = "depthai_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn depthai_ros_msgs__msg__HandLandmarkArray__init(msg: *mut HandLandmarkArray) -> bool;
    fn depthai_ros_msgs__msg__HandLandmarkArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<HandLandmarkArray>, size: usize) -> bool;
    fn depthai_ros_msgs__msg__HandLandmarkArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<HandLandmarkArray>);
    fn depthai_ros_msgs__msg__HandLandmarkArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<HandLandmarkArray>, out_seq: *mut rosidl_runtime_rs::Sequence<HandLandmarkArray>) -> bool;
}

// Corresponds to depthai_ros_msgs__msg__HandLandmarkArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// A list of hand landmark detections

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct HandLandmarkArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// A list of the detected proposals. A multi-proposal detector might generate along with the 3D depth information
    ///   this list with many candidate detections generated from a single input.
    pub landmarks: rosidl_runtime_rs::Sequence<super::super::msg::rmw::HandLandmark>,

}



impl Default for HandLandmarkArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !depthai_ros_msgs__msg__HandLandmarkArray__init(&mut msg as *mut _) {
        panic!("Call to depthai_ros_msgs__msg__HandLandmarkArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for HandLandmarkArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__HandLandmarkArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__HandLandmarkArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__HandLandmarkArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for HandLandmarkArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for HandLandmarkArray where Self: Sized {
  const TYPE_NAME: &'static str = "depthai_ros_msgs/msg/HandLandmarkArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__HandLandmarkArray() }
  }
}


#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__ImuWithMagneticField() -> *const std::ffi::c_void;
}

#[link(name = "depthai_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn depthai_ros_msgs__msg__ImuWithMagneticField__init(msg: *mut ImuWithMagneticField) -> bool;
    fn depthai_ros_msgs__msg__ImuWithMagneticField__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ImuWithMagneticField>, size: usize) -> bool;
    fn depthai_ros_msgs__msg__ImuWithMagneticField__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ImuWithMagneticField>);
    fn depthai_ros_msgs__msg__ImuWithMagneticField__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ImuWithMagneticField>, out_seq: *mut rosidl_runtime_rs::Sequence<ImuWithMagneticField>) -> bool;
}

// Corresponds to depthai_ros_msgs__msg__ImuWithMagneticField
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ImuWithMagneticField {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub imu: sensor_msgs::msg::rmw::Imu,


    // This member is not documented.
    #[allow(missing_docs)]
    pub field: sensor_msgs::msg::rmw::MagneticField,

}



impl Default for ImuWithMagneticField {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !depthai_ros_msgs__msg__ImuWithMagneticField__init(&mut msg as *mut _) {
        panic!("Call to depthai_ros_msgs__msg__ImuWithMagneticField__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ImuWithMagneticField {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__ImuWithMagneticField__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__ImuWithMagneticField__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__ImuWithMagneticField__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ImuWithMagneticField {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ImuWithMagneticField where Self: Sized {
  const TYPE_NAME: &'static str = "depthai_ros_msgs/msg/ImuWithMagneticField";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__ImuWithMagneticField() }
  }
}


#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__TrackedFeature() -> *const std::ffi::c_void;
}

#[link(name = "depthai_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn depthai_ros_msgs__msg__TrackedFeature__init(msg: *mut TrackedFeature) -> bool;
    fn depthai_ros_msgs__msg__TrackedFeature__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TrackedFeature>, size: usize) -> bool;
    fn depthai_ros_msgs__msg__TrackedFeature__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TrackedFeature>);
    fn depthai_ros_msgs__msg__TrackedFeature__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TrackedFeature>, out_seq: *mut rosidl_runtime_rs::Sequence<TrackedFeature>) -> bool;
}

// Corresponds to depthai_ros_msgs__msg__TrackedFeature
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrackedFeature {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::rmw::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub id: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub age: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub harris_score: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tracking_error: f32,

}



impl Default for TrackedFeature {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !depthai_ros_msgs__msg__TrackedFeature__init(&mut msg as *mut _) {
        panic!("Call to depthai_ros_msgs__msg__TrackedFeature__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TrackedFeature {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__TrackedFeature__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__TrackedFeature__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__TrackedFeature__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TrackedFeature {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TrackedFeature where Self: Sized {
  const TYPE_NAME: &'static str = "depthai_ros_msgs/msg/TrackedFeature";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__TrackedFeature() }
  }
}


#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__TrackedFeatures() -> *const std::ffi::c_void;
}

#[link(name = "depthai_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn depthai_ros_msgs__msg__TrackedFeatures__init(msg: *mut TrackedFeatures) -> bool;
    fn depthai_ros_msgs__msg__TrackedFeatures__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TrackedFeatures>, size: usize) -> bool;
    fn depthai_ros_msgs__msg__TrackedFeatures__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TrackedFeatures>);
    fn depthai_ros_msgs__msg__TrackedFeatures__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TrackedFeatures>, out_seq: *mut rosidl_runtime_rs::Sequence<TrackedFeatures>) -> bool;
}

// Corresponds to depthai_ros_msgs__msg__TrackedFeatures
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrackedFeatures {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub features: rosidl_runtime_rs::Sequence<super::super::msg::rmw::TrackedFeature>,

}



impl Default for TrackedFeatures {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !depthai_ros_msgs__msg__TrackedFeatures__init(&mut msg as *mut _) {
        panic!("Call to depthai_ros_msgs__msg__TrackedFeatures__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TrackedFeatures {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__TrackedFeatures__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__TrackedFeatures__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__TrackedFeatures__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TrackedFeatures {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TrackedFeatures where Self: Sized {
  const TYPE_NAME: &'static str = "depthai_ros_msgs/msg/TrackedFeatures";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__TrackedFeatures() }
  }
}


#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__SpatialDetection() -> *const std::ffi::c_void;
}

#[link(name = "depthai_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn depthai_ros_msgs__msg__SpatialDetection__init(msg: *mut SpatialDetection) -> bool;
    fn depthai_ros_msgs__msg__SpatialDetection__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SpatialDetection>, size: usize) -> bool;
    fn depthai_ros_msgs__msg__SpatialDetection__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SpatialDetection>);
    fn depthai_ros_msgs__msg__SpatialDetection__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SpatialDetection>, out_seq: *mut rosidl_runtime_rs::Sequence<SpatialDetection>) -> bool;
}

// Corresponds to depthai_ros_msgs__msg__SpatialDetection
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SpatialDetection {
    /// Class probabilities
    pub results: rosidl_runtime_rs::Sequence<vision_msgs::msg::rmw::ObjectHypothesis>,

    /// 2D bounding box surrounding the object.
    pub bbox: vision_msgs::msg::rmw::BoundingBox2D,

    /// Center of the detected object in meters
    pub position: geometry_msgs::msg::rmw::Point,

    /// If true, this message contains object tracking information.
    pub is_tracking: bool,

    /// ID used for consistency across multiple detection messages. This value will
    /// likely differ from the id field set in each individual ObjectHypothesis.
    /// If you set this field, be sure to also set is_tracking to True.
    pub tracking_id: rosidl_runtime_rs::String,

}



impl Default for SpatialDetection {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !depthai_ros_msgs__msg__SpatialDetection__init(&mut msg as *mut _) {
        panic!("Call to depthai_ros_msgs__msg__SpatialDetection__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SpatialDetection {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__SpatialDetection__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__SpatialDetection__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__SpatialDetection__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SpatialDetection {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SpatialDetection where Self: Sized {
  const TYPE_NAME: &'static str = "depthai_ros_msgs/msg/SpatialDetection";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__SpatialDetection() }
  }
}


#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__SpatialDetectionArray() -> *const std::ffi::c_void;
}

#[link(name = "depthai_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn depthai_ros_msgs__msg__SpatialDetectionArray__init(msg: *mut SpatialDetectionArray) -> bool;
    fn depthai_ros_msgs__msg__SpatialDetectionArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SpatialDetectionArray>, size: usize) -> bool;
    fn depthai_ros_msgs__msg__SpatialDetectionArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SpatialDetectionArray>);
    fn depthai_ros_msgs__msg__SpatialDetectionArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SpatialDetectionArray>, out_seq: *mut rosidl_runtime_rs::Sequence<SpatialDetectionArray>) -> bool;
}

// Corresponds to depthai_ros_msgs__msg__SpatialDetectionArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// A list of 2D detections, for a multi-object 2D detector along with 3D depth information.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SpatialDetectionArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// A list of the detected proposals. A multi-proposal detector might generate along with the 3D depth information
    ///   this list with many candidate detections generated from a single input.
    pub detections: rosidl_runtime_rs::Sequence<super::super::msg::rmw::SpatialDetection>,

}



impl Default for SpatialDetectionArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !depthai_ros_msgs__msg__SpatialDetectionArray__init(&mut msg as *mut _) {
        panic!("Call to depthai_ros_msgs__msg__SpatialDetectionArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SpatialDetectionArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__SpatialDetectionArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__SpatialDetectionArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__SpatialDetectionArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SpatialDetectionArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SpatialDetectionArray where Self: Sized {
  const TYPE_NAME: &'static str = "depthai_ros_msgs/msg/SpatialDetectionArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__SpatialDetectionArray() }
  }
}


#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__TrackDetection2D() -> *const std::ffi::c_void;
}

#[link(name = "depthai_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn depthai_ros_msgs__msg__TrackDetection2D__init(msg: *mut TrackDetection2D) -> bool;
    fn depthai_ros_msgs__msg__TrackDetection2D__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TrackDetection2D>, size: usize) -> bool;
    fn depthai_ros_msgs__msg__TrackDetection2D__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TrackDetection2D>);
    fn depthai_ros_msgs__msg__TrackDetection2D__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TrackDetection2D>, out_seq: *mut rosidl_runtime_rs::Sequence<TrackDetection2D>) -> bool;
}

// Corresponds to depthai_ros_msgs__msg__TrackDetection2D
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrackDetection2D {
    /// Class probabilities
    pub results: rosidl_runtime_rs::Sequence<vision_msgs::msg::rmw::ObjectHypothesisWithPose>,

    /// 2D bounding box surrounding the object.
    pub bbox: vision_msgs::msg::rmw::BoundingBox2D,

    /// If true, this message contains object tracking information.
    pub is_tracking: bool,

    /// ID used for consistency across multiple detection messages. This value will
    /// likely differ from the id field set in each individual ObjectHypothesis.
    /// If you set this field, be sure to also set is_tracking to True.
    pub tracking_id: rosidl_runtime_rs::String,

    /// Age: number of frames the object is being tracked
    pub tracking_age: i32,

    /// Status of the tracking:
    /// 0 = NEW -> the object is newly added.
    /// 1 = TRACKED -> the object is being tracked.
    /// 2 = LOST -> the object gets lost now. The object can be tracked again automatically (long term tracking)
    ///     or by specifying detected object manually (short term and zero term tracking).
    /// 3 = REMOVED -> the object is removed.
    pub tracking_status: i32,

}



impl Default for TrackDetection2D {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !depthai_ros_msgs__msg__TrackDetection2D__init(&mut msg as *mut _) {
        panic!("Call to depthai_ros_msgs__msg__TrackDetection2D__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TrackDetection2D {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__TrackDetection2D__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__TrackDetection2D__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__TrackDetection2D__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TrackDetection2D {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TrackDetection2D where Self: Sized {
  const TYPE_NAME: &'static str = "depthai_ros_msgs/msg/TrackDetection2D";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__TrackDetection2D() }
  }
}


#[link(name = "depthai_ros_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__TrackDetection2DArray() -> *const std::ffi::c_void;
}

#[link(name = "depthai_ros_msgs__rosidl_generator_c")]
extern "C" {
    fn depthai_ros_msgs__msg__TrackDetection2DArray__init(msg: *mut TrackDetection2DArray) -> bool;
    fn depthai_ros_msgs__msg__TrackDetection2DArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TrackDetection2DArray>, size: usize) -> bool;
    fn depthai_ros_msgs__msg__TrackDetection2DArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TrackDetection2DArray>);
    fn depthai_ros_msgs__msg__TrackDetection2DArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TrackDetection2DArray>, out_seq: *mut rosidl_runtime_rs::Sequence<TrackDetection2DArray>) -> bool;
}

// Corresponds to depthai_ros_msgs__msg__TrackDetection2DArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// A list of 2D tracklets, for a multi-object 2D tracker.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TrackDetection2DArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// A list of the tracking proposals.
    pub detections: rosidl_runtime_rs::Sequence<super::super::msg::rmw::TrackDetection2D>,

}



impl Default for TrackDetection2DArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !depthai_ros_msgs__msg__TrackDetection2DArray__init(&mut msg as *mut _) {
        panic!("Call to depthai_ros_msgs__msg__TrackDetection2DArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TrackDetection2DArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__TrackDetection2DArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__TrackDetection2DArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { depthai_ros_msgs__msg__TrackDetection2DArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TrackDetection2DArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TrackDetection2DArray where Self: Sized {
  const TYPE_NAME: &'static str = "depthai_ros_msgs/msg/TrackDetection2DArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__depthai_ros_msgs__msg__TrackDetection2DArray() }
  }
}


