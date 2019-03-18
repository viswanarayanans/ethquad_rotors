// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ConnectRosToGazeboTopic.proto

#ifndef PROTOBUF_ConnectRosToGazeboTopic_2eproto__INCLUDED
#define PROTOBUF_ConnectRosToGazeboTopic_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace gz_std_msgs {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_ConnectRosToGazeboTopic_2eproto();
void protobuf_AssignDesc_ConnectRosToGazeboTopic_2eproto();
void protobuf_ShutdownFile_ConnectRosToGazeboTopic_2eproto();

class ConnectRosToGazeboTopic;

enum ConnectRosToGazeboTopic_MsgType {
  ConnectRosToGazeboTopic_MsgType_ACTUATORS = 0,
  ConnectRosToGazeboTopic_MsgType_COMMAND_MOTOR_SPEED = 1,
  ConnectRosToGazeboTopic_MsgType_ROLL_PITCH_YAWRATE_THRUST = 2,
  ConnectRosToGazeboTopic_MsgType_WIND_SPEED = 3
};
bool ConnectRosToGazeboTopic_MsgType_IsValid(int value);
const ConnectRosToGazeboTopic_MsgType ConnectRosToGazeboTopic_MsgType_MsgType_MIN = ConnectRosToGazeboTopic_MsgType_ACTUATORS;
const ConnectRosToGazeboTopic_MsgType ConnectRosToGazeboTopic_MsgType_MsgType_MAX = ConnectRosToGazeboTopic_MsgType_WIND_SPEED;
const int ConnectRosToGazeboTopic_MsgType_MsgType_ARRAYSIZE = ConnectRosToGazeboTopic_MsgType_MsgType_MAX + 1;

const ::google::protobuf::EnumDescriptor* ConnectRosToGazeboTopic_MsgType_descriptor();
inline const ::std::string& ConnectRosToGazeboTopic_MsgType_Name(ConnectRosToGazeboTopic_MsgType value) {
  return ::google::protobuf::internal::NameOfEnum(
    ConnectRosToGazeboTopic_MsgType_descriptor(), value);
}
inline bool ConnectRosToGazeboTopic_MsgType_Parse(
    const ::std::string& name, ConnectRosToGazeboTopic_MsgType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<ConnectRosToGazeboTopic_MsgType>(
    ConnectRosToGazeboTopic_MsgType_descriptor(), name, value);
}
// ===================================================================

class ConnectRosToGazeboTopic : public ::google::protobuf::Message {
 public:
  ConnectRosToGazeboTopic();
  virtual ~ConnectRosToGazeboTopic();

  ConnectRosToGazeboTopic(const ConnectRosToGazeboTopic& from);

  inline ConnectRosToGazeboTopic& operator=(const ConnectRosToGazeboTopic& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const ConnectRosToGazeboTopic& default_instance();

  void Swap(ConnectRosToGazeboTopic* other);

  // implements Message ----------------------------------------------

  ConnectRosToGazeboTopic* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const ConnectRosToGazeboTopic& from);
  void MergeFrom(const ConnectRosToGazeboTopic& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  typedef ConnectRosToGazeboTopic_MsgType MsgType;
  static const MsgType ACTUATORS = ConnectRosToGazeboTopic_MsgType_ACTUATORS;
  static const MsgType COMMAND_MOTOR_SPEED = ConnectRosToGazeboTopic_MsgType_COMMAND_MOTOR_SPEED;
  static const MsgType ROLL_PITCH_YAWRATE_THRUST = ConnectRosToGazeboTopic_MsgType_ROLL_PITCH_YAWRATE_THRUST;
  static const MsgType WIND_SPEED = ConnectRosToGazeboTopic_MsgType_WIND_SPEED;
  static inline bool MsgType_IsValid(int value) {
    return ConnectRosToGazeboTopic_MsgType_IsValid(value);
  }
  static const MsgType MsgType_MIN =
    ConnectRosToGazeboTopic_MsgType_MsgType_MIN;
  static const MsgType MsgType_MAX =
    ConnectRosToGazeboTopic_MsgType_MsgType_MAX;
  static const int MsgType_ARRAYSIZE =
    ConnectRosToGazeboTopic_MsgType_MsgType_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  MsgType_descriptor() {
    return ConnectRosToGazeboTopic_MsgType_descriptor();
  }
  static inline const ::std::string& MsgType_Name(MsgType value) {
    return ConnectRosToGazeboTopic_MsgType_Name(value);
  }
  static inline bool MsgType_Parse(const ::std::string& name,
      MsgType* value) {
    return ConnectRosToGazeboTopic_MsgType_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // required string ros_topic = 1;
  inline bool has_ros_topic() const;
  inline void clear_ros_topic();
  static const int kRosTopicFieldNumber = 1;
  inline const ::std::string& ros_topic() const;
  inline void set_ros_topic(const ::std::string& value);
  inline void set_ros_topic(const char* value);
  inline void set_ros_topic(const char* value, size_t size);
  inline ::std::string* mutable_ros_topic();
  inline ::std::string* release_ros_topic();
  inline void set_allocated_ros_topic(::std::string* ros_topic);

  // required string gazebo_topic = 3;
  inline bool has_gazebo_topic() const;
  inline void clear_gazebo_topic();
  static const int kGazeboTopicFieldNumber = 3;
  inline const ::std::string& gazebo_topic() const;
  inline void set_gazebo_topic(const ::std::string& value);
  inline void set_gazebo_topic(const char* value);
  inline void set_gazebo_topic(const char* value, size_t size);
  inline ::std::string* mutable_gazebo_topic();
  inline ::std::string* release_gazebo_topic();
  inline void set_allocated_gazebo_topic(::std::string* gazebo_topic);

  // required .gz_std_msgs.ConnectRosToGazeboTopic.MsgType msgType = 4;
  inline bool has_msgtype() const;
  inline void clear_msgtype();
  static const int kMsgTypeFieldNumber = 4;
  inline ::gz_std_msgs::ConnectRosToGazeboTopic_MsgType msgtype() const;
  inline void set_msgtype(::gz_std_msgs::ConnectRosToGazeboTopic_MsgType value);

  // @@protoc_insertion_point(class_scope:gz_std_msgs.ConnectRosToGazeboTopic)
 private:
  inline void set_has_ros_topic();
  inline void clear_has_ros_topic();
  inline void set_has_gazebo_topic();
  inline void clear_has_gazebo_topic();
  inline void set_has_msgtype();
  inline void clear_has_msgtype();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::std::string* ros_topic_;
  ::std::string* gazebo_topic_;
  int msgtype_;
  friend void  protobuf_AddDesc_ConnectRosToGazeboTopic_2eproto();
  friend void protobuf_AssignDesc_ConnectRosToGazeboTopic_2eproto();
  friend void protobuf_ShutdownFile_ConnectRosToGazeboTopic_2eproto();

  void InitAsDefaultInstance();
  static ConnectRosToGazeboTopic* default_instance_;
};
// ===================================================================


// ===================================================================

// ConnectRosToGazeboTopic

// required string ros_topic = 1;
inline bool ConnectRosToGazeboTopic::has_ros_topic() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ConnectRosToGazeboTopic::set_has_ros_topic() {
  _has_bits_[0] |= 0x00000001u;
}
inline void ConnectRosToGazeboTopic::clear_has_ros_topic() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void ConnectRosToGazeboTopic::clear_ros_topic() {
  if (ros_topic_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    ros_topic_->clear();
  }
  clear_has_ros_topic();
}
inline const ::std::string& ConnectRosToGazeboTopic::ros_topic() const {
  // @@protoc_insertion_point(field_get:gz_std_msgs.ConnectRosToGazeboTopic.ros_topic)
  return *ros_topic_;
}
inline void ConnectRosToGazeboTopic::set_ros_topic(const ::std::string& value) {
  set_has_ros_topic();
  if (ros_topic_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    ros_topic_ = new ::std::string;
  }
  ros_topic_->assign(value);
  // @@protoc_insertion_point(field_set:gz_std_msgs.ConnectRosToGazeboTopic.ros_topic)
}
inline void ConnectRosToGazeboTopic::set_ros_topic(const char* value) {
  set_has_ros_topic();
  if (ros_topic_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    ros_topic_ = new ::std::string;
  }
  ros_topic_->assign(value);
  // @@protoc_insertion_point(field_set_char:gz_std_msgs.ConnectRosToGazeboTopic.ros_topic)
}
inline void ConnectRosToGazeboTopic::set_ros_topic(const char* value, size_t size) {
  set_has_ros_topic();
  if (ros_topic_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    ros_topic_ = new ::std::string;
  }
  ros_topic_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:gz_std_msgs.ConnectRosToGazeboTopic.ros_topic)
}
inline ::std::string* ConnectRosToGazeboTopic::mutable_ros_topic() {
  set_has_ros_topic();
  if (ros_topic_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    ros_topic_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:gz_std_msgs.ConnectRosToGazeboTopic.ros_topic)
  return ros_topic_;
}
inline ::std::string* ConnectRosToGazeboTopic::release_ros_topic() {
  clear_has_ros_topic();
  if (ros_topic_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = ros_topic_;
    ros_topic_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void ConnectRosToGazeboTopic::set_allocated_ros_topic(::std::string* ros_topic) {
  if (ros_topic_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete ros_topic_;
  }
  if (ros_topic) {
    set_has_ros_topic();
    ros_topic_ = ros_topic;
  } else {
    clear_has_ros_topic();
    ros_topic_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:gz_std_msgs.ConnectRosToGazeboTopic.ros_topic)
}

// required string gazebo_topic = 3;
inline bool ConnectRosToGazeboTopic::has_gazebo_topic() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ConnectRosToGazeboTopic::set_has_gazebo_topic() {
  _has_bits_[0] |= 0x00000002u;
}
inline void ConnectRosToGazeboTopic::clear_has_gazebo_topic() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void ConnectRosToGazeboTopic::clear_gazebo_topic() {
  if (gazebo_topic_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    gazebo_topic_->clear();
  }
  clear_has_gazebo_topic();
}
inline const ::std::string& ConnectRosToGazeboTopic::gazebo_topic() const {
  // @@protoc_insertion_point(field_get:gz_std_msgs.ConnectRosToGazeboTopic.gazebo_topic)
  return *gazebo_topic_;
}
inline void ConnectRosToGazeboTopic::set_gazebo_topic(const ::std::string& value) {
  set_has_gazebo_topic();
  if (gazebo_topic_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    gazebo_topic_ = new ::std::string;
  }
  gazebo_topic_->assign(value);
  // @@protoc_insertion_point(field_set:gz_std_msgs.ConnectRosToGazeboTopic.gazebo_topic)
}
inline void ConnectRosToGazeboTopic::set_gazebo_topic(const char* value) {
  set_has_gazebo_topic();
  if (gazebo_topic_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    gazebo_topic_ = new ::std::string;
  }
  gazebo_topic_->assign(value);
  // @@protoc_insertion_point(field_set_char:gz_std_msgs.ConnectRosToGazeboTopic.gazebo_topic)
}
inline void ConnectRosToGazeboTopic::set_gazebo_topic(const char* value, size_t size) {
  set_has_gazebo_topic();
  if (gazebo_topic_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    gazebo_topic_ = new ::std::string;
  }
  gazebo_topic_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:gz_std_msgs.ConnectRosToGazeboTopic.gazebo_topic)
}
inline ::std::string* ConnectRosToGazeboTopic::mutable_gazebo_topic() {
  set_has_gazebo_topic();
  if (gazebo_topic_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    gazebo_topic_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:gz_std_msgs.ConnectRosToGazeboTopic.gazebo_topic)
  return gazebo_topic_;
}
inline ::std::string* ConnectRosToGazeboTopic::release_gazebo_topic() {
  clear_has_gazebo_topic();
  if (gazebo_topic_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = gazebo_topic_;
    gazebo_topic_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void ConnectRosToGazeboTopic::set_allocated_gazebo_topic(::std::string* gazebo_topic) {
  if (gazebo_topic_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete gazebo_topic_;
  }
  if (gazebo_topic) {
    set_has_gazebo_topic();
    gazebo_topic_ = gazebo_topic;
  } else {
    clear_has_gazebo_topic();
    gazebo_topic_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:gz_std_msgs.ConnectRosToGazeboTopic.gazebo_topic)
}

// required .gz_std_msgs.ConnectRosToGazeboTopic.MsgType msgType = 4;
inline bool ConnectRosToGazeboTopic::has_msgtype() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void ConnectRosToGazeboTopic::set_has_msgtype() {
  _has_bits_[0] |= 0x00000004u;
}
inline void ConnectRosToGazeboTopic::clear_has_msgtype() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void ConnectRosToGazeboTopic::clear_msgtype() {
  msgtype_ = 0;
  clear_has_msgtype();
}
inline ::gz_std_msgs::ConnectRosToGazeboTopic_MsgType ConnectRosToGazeboTopic::msgtype() const {
  // @@protoc_insertion_point(field_get:gz_std_msgs.ConnectRosToGazeboTopic.msgType)
  return static_cast< ::gz_std_msgs::ConnectRosToGazeboTopic_MsgType >(msgtype_);
}
inline void ConnectRosToGazeboTopic::set_msgtype(::gz_std_msgs::ConnectRosToGazeboTopic_MsgType value) {
  assert(::gz_std_msgs::ConnectRosToGazeboTopic_MsgType_IsValid(value));
  set_has_msgtype();
  msgtype_ = value;
  // @@protoc_insertion_point(field_set:gz_std_msgs.ConnectRosToGazeboTopic.msgType)
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace gz_std_msgs

#ifndef SWIG
namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::gz_std_msgs::ConnectRosToGazeboTopic_MsgType> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::gz_std_msgs::ConnectRosToGazeboTopic_MsgType>() {
  return ::gz_std_msgs::ConnectRosToGazeboTopic_MsgType_descriptor();
}

}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_ConnectRosToGazeboTopic_2eproto__INCLUDED