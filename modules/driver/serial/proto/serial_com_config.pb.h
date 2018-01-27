// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: serial_com_config.proto

#ifndef PROTOBUF_serial_5fcom_5fconfig_2eproto_INCLUDED
#define PROTOBUF_serial_5fcom_5fconfig_2eproto_INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3005000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace protobuf_serial_5fcom_5fconfig_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultsSerialPortConfigImpl();
void InitDefaultsSerialPortConfig();
inline void InitDefaults() {
  InitDefaultsSerialPortConfig();
}
}  // namespace protobuf_serial_5fcom_5fconfig_2eproto
namespace rrts {
namespace driver {
namespace serial {
class SerialPortConfig;
class SerialPortConfigDefaultTypeInternal;
extern SerialPortConfigDefaultTypeInternal _SerialPortConfig_default_instance_;
}  // namespace serial
}  // namespace driver
}  // namespace rrts
namespace google {
namespace protobuf {
template<> ::rrts::driver::serial::SerialPortConfig* Arena::Create< ::rrts::driver::serial::SerialPortConfig>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace rrts {
namespace driver {
namespace serial {

// ===================================================================

class SerialPortConfig : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:rrts.driver.serial.SerialPortConfig) */ {
 public:
  SerialPortConfig();
  virtual ~SerialPortConfig();

  SerialPortConfig(const SerialPortConfig& from);

  inline SerialPortConfig& operator=(const SerialPortConfig& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  SerialPortConfig(SerialPortConfig&& from) noexcept
    : SerialPortConfig() {
    *this = ::std::move(from);
  }

  inline SerialPortConfig& operator=(SerialPortConfig&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const SerialPortConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SerialPortConfig* internal_default_instance() {
    return reinterpret_cast<const SerialPortConfig*>(
               &_SerialPortConfig_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(SerialPortConfig* other);
  friend void swap(SerialPortConfig& a, SerialPortConfig& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline SerialPortConfig* New() const PROTOBUF_FINAL {
    return ::google::protobuf::Arena::Create<SerialPortConfig>(NULL);
  }

  SerialPortConfig* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL {
    return ::google::protobuf::Arena::Create<SerialPortConfig>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const SerialPortConfig& from);
  void MergeFrom(const SerialPortConfig& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(SerialPortConfig* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required string serial_port = 1;
  bool has_serial_port() const;
  void clear_serial_port();
  static const int kSerialPortFieldNumber = 1;
  const ::std::string& serial_port() const;
  void set_serial_port(const ::std::string& value);
  #if LANG_CXX11
  void set_serial_port(::std::string&& value);
  #endif
  void set_serial_port(const char* value);
  void set_serial_port(const char* value, size_t size);
  ::std::string* mutable_serial_port();
  ::std::string* release_serial_port();
  void set_allocated_serial_port(::std::string* serial_port);

  // optional string parity = 6 [default = "N"];
  bool has_parity() const;
  void clear_parity();
  static const int kParityFieldNumber = 6;
  const ::std::string& parity() const;
  void set_parity(const ::std::string& value);
  #if LANG_CXX11
  void set_parity(::std::string&& value);
  #endif
  void set_parity(const char* value);
  void set_parity(const char* value, size_t size);
  ::std::string* mutable_parity();
  ::std::string* release_parity();
  void set_allocated_parity(::std::string* parity);

  // required int32 serial_boudrate = 2;
  bool has_serial_boudrate() const;
  void clear_serial_boudrate();
  static const int kSerialBoudrateFieldNumber = 2;
  ::google::protobuf::int32 serial_boudrate() const;
  void set_serial_boudrate(::google::protobuf::int32 value);

  // optional int32 flow_control = 3 [default = 0];
  bool has_flow_control() const;
  void clear_flow_control();
  static const int kFlowControlFieldNumber = 3;
  ::google::protobuf::int32 flow_control() const;
  void set_flow_control(::google::protobuf::int32 value);

  // optional bool is_debug = 7 [default = true];
  bool has_is_debug() const;
  void clear_is_debug();
  static const int kIsDebugFieldNumber = 7;
  bool is_debug() const;
  void set_is_debug(bool value);

  // optional int32 databits = 4 [default = 8];
  bool has_databits() const;
  void clear_databits();
  static const int kDatabitsFieldNumber = 4;
  ::google::protobuf::int32 databits() const;
  void set_databits(::google::protobuf::int32 value);

  // optional int32 stopbits = 5 [default = 1];
  bool has_stopbits() const;
  void clear_stopbits();
  static const int kStopbitsFieldNumber = 5;
  ::google::protobuf::int32 stopbits() const;
  void set_stopbits(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:rrts.driver.serial.SerialPortConfig)
 private:
  void set_has_serial_port();
  void clear_has_serial_port();
  void set_has_serial_boudrate();
  void clear_has_serial_boudrate();
  void set_has_flow_control();
  void clear_has_flow_control();
  void set_has_databits();
  void clear_has_databits();
  void set_has_stopbits();
  void clear_has_stopbits();
  void set_has_parity();
  void clear_has_parity();
  void set_has_is_debug();
  void clear_has_is_debug();

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr serial_port_;
  static ::google::protobuf::internal::ExplicitlyConstructed< ::std::string> _default_parity_;
  ::google::protobuf::internal::ArenaStringPtr parity_;
  ::google::protobuf::int32 serial_boudrate_;
  ::google::protobuf::int32 flow_control_;
  bool is_debug_;
  ::google::protobuf::int32 databits_;
  ::google::protobuf::int32 stopbits_;
  friend struct ::protobuf_serial_5fcom_5fconfig_2eproto::TableStruct;
  friend void ::protobuf_serial_5fcom_5fconfig_2eproto::InitDefaultsSerialPortConfigImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SerialPortConfig

// required string serial_port = 1;
inline bool SerialPortConfig::has_serial_port() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SerialPortConfig::set_has_serial_port() {
  _has_bits_[0] |= 0x00000001u;
}
inline void SerialPortConfig::clear_has_serial_port() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void SerialPortConfig::clear_serial_port() {
  serial_port_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_serial_port();
}
inline const ::std::string& SerialPortConfig::serial_port() const {
  // @@protoc_insertion_point(field_get:rrts.driver.serial.SerialPortConfig.serial_port)
  return serial_port_.GetNoArena();
}
inline void SerialPortConfig::set_serial_port(const ::std::string& value) {
  set_has_serial_port();
  serial_port_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:rrts.driver.serial.SerialPortConfig.serial_port)
}
#if LANG_CXX11
inline void SerialPortConfig::set_serial_port(::std::string&& value) {
  set_has_serial_port();
  serial_port_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:rrts.driver.serial.SerialPortConfig.serial_port)
}
#endif
inline void SerialPortConfig::set_serial_port(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_serial_port();
  serial_port_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:rrts.driver.serial.SerialPortConfig.serial_port)
}
inline void SerialPortConfig::set_serial_port(const char* value, size_t size) {
  set_has_serial_port();
  serial_port_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:rrts.driver.serial.SerialPortConfig.serial_port)
}
inline ::std::string* SerialPortConfig::mutable_serial_port() {
  set_has_serial_port();
  // @@protoc_insertion_point(field_mutable:rrts.driver.serial.SerialPortConfig.serial_port)
  return serial_port_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* SerialPortConfig::release_serial_port() {
  // @@protoc_insertion_point(field_release:rrts.driver.serial.SerialPortConfig.serial_port)
  clear_has_serial_port();
  return serial_port_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void SerialPortConfig::set_allocated_serial_port(::std::string* serial_port) {
  if (serial_port != NULL) {
    set_has_serial_port();
  } else {
    clear_has_serial_port();
  }
  serial_port_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), serial_port);
  // @@protoc_insertion_point(field_set_allocated:rrts.driver.serial.SerialPortConfig.serial_port)
}

// required int32 serial_boudrate = 2;
inline bool SerialPortConfig::has_serial_boudrate() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void SerialPortConfig::set_has_serial_boudrate() {
  _has_bits_[0] |= 0x00000004u;
}
inline void SerialPortConfig::clear_has_serial_boudrate() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void SerialPortConfig::clear_serial_boudrate() {
  serial_boudrate_ = 0;
  clear_has_serial_boudrate();
}
inline ::google::protobuf::int32 SerialPortConfig::serial_boudrate() const {
  // @@protoc_insertion_point(field_get:rrts.driver.serial.SerialPortConfig.serial_boudrate)
  return serial_boudrate_;
}
inline void SerialPortConfig::set_serial_boudrate(::google::protobuf::int32 value) {
  set_has_serial_boudrate();
  serial_boudrate_ = value;
  // @@protoc_insertion_point(field_set:rrts.driver.serial.SerialPortConfig.serial_boudrate)
}

// optional int32 flow_control = 3 [default = 0];
inline bool SerialPortConfig::has_flow_control() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void SerialPortConfig::set_has_flow_control() {
  _has_bits_[0] |= 0x00000008u;
}
inline void SerialPortConfig::clear_has_flow_control() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void SerialPortConfig::clear_flow_control() {
  flow_control_ = 0;
  clear_has_flow_control();
}
inline ::google::protobuf::int32 SerialPortConfig::flow_control() const {
  // @@protoc_insertion_point(field_get:rrts.driver.serial.SerialPortConfig.flow_control)
  return flow_control_;
}
inline void SerialPortConfig::set_flow_control(::google::protobuf::int32 value) {
  set_has_flow_control();
  flow_control_ = value;
  // @@protoc_insertion_point(field_set:rrts.driver.serial.SerialPortConfig.flow_control)
}

// optional int32 databits = 4 [default = 8];
inline bool SerialPortConfig::has_databits() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void SerialPortConfig::set_has_databits() {
  _has_bits_[0] |= 0x00000020u;
}
inline void SerialPortConfig::clear_has_databits() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void SerialPortConfig::clear_databits() {
  databits_ = 8;
  clear_has_databits();
}
inline ::google::protobuf::int32 SerialPortConfig::databits() const {
  // @@protoc_insertion_point(field_get:rrts.driver.serial.SerialPortConfig.databits)
  return databits_;
}
inline void SerialPortConfig::set_databits(::google::protobuf::int32 value) {
  set_has_databits();
  databits_ = value;
  // @@protoc_insertion_point(field_set:rrts.driver.serial.SerialPortConfig.databits)
}

// optional int32 stopbits = 5 [default = 1];
inline bool SerialPortConfig::has_stopbits() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void SerialPortConfig::set_has_stopbits() {
  _has_bits_[0] |= 0x00000040u;
}
inline void SerialPortConfig::clear_has_stopbits() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void SerialPortConfig::clear_stopbits() {
  stopbits_ = 1;
  clear_has_stopbits();
}
inline ::google::protobuf::int32 SerialPortConfig::stopbits() const {
  // @@protoc_insertion_point(field_get:rrts.driver.serial.SerialPortConfig.stopbits)
  return stopbits_;
}
inline void SerialPortConfig::set_stopbits(::google::protobuf::int32 value) {
  set_has_stopbits();
  stopbits_ = value;
  // @@protoc_insertion_point(field_set:rrts.driver.serial.SerialPortConfig.stopbits)
}

// optional string parity = 6 [default = "N"];
inline bool SerialPortConfig::has_parity() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void SerialPortConfig::set_has_parity() {
  _has_bits_[0] |= 0x00000002u;
}
inline void SerialPortConfig::clear_has_parity() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void SerialPortConfig::clear_parity() {
  parity_.ClearToDefaultNoArena(&::rrts::driver::serial::SerialPortConfig::_default_parity_.get());
  clear_has_parity();
}
inline const ::std::string& SerialPortConfig::parity() const {
  // @@protoc_insertion_point(field_get:rrts.driver.serial.SerialPortConfig.parity)
  return parity_.GetNoArena();
}
inline void SerialPortConfig::set_parity(const ::std::string& value) {
  set_has_parity();
  parity_.SetNoArena(&::rrts::driver::serial::SerialPortConfig::_default_parity_.get(), value);
  // @@protoc_insertion_point(field_set:rrts.driver.serial.SerialPortConfig.parity)
}
#if LANG_CXX11
inline void SerialPortConfig::set_parity(::std::string&& value) {
  set_has_parity();
  parity_.SetNoArena(
    &::rrts::driver::serial::SerialPortConfig::_default_parity_.get(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:rrts.driver.serial.SerialPortConfig.parity)
}
#endif
inline void SerialPortConfig::set_parity(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_parity();
  parity_.SetNoArena(&::rrts::driver::serial::SerialPortConfig::_default_parity_.get(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:rrts.driver.serial.SerialPortConfig.parity)
}
inline void SerialPortConfig::set_parity(const char* value, size_t size) {
  set_has_parity();
  parity_.SetNoArena(&::rrts::driver::serial::SerialPortConfig::_default_parity_.get(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:rrts.driver.serial.SerialPortConfig.parity)
}
inline ::std::string* SerialPortConfig::mutable_parity() {
  set_has_parity();
  // @@protoc_insertion_point(field_mutable:rrts.driver.serial.SerialPortConfig.parity)
  return parity_.MutableNoArena(&::rrts::driver::serial::SerialPortConfig::_default_parity_.get());
}
inline ::std::string* SerialPortConfig::release_parity() {
  // @@protoc_insertion_point(field_release:rrts.driver.serial.SerialPortConfig.parity)
  clear_has_parity();
  return parity_.ReleaseNoArena(&::rrts::driver::serial::SerialPortConfig::_default_parity_.get());
}
inline void SerialPortConfig::set_allocated_parity(::std::string* parity) {
  if (parity != NULL) {
    set_has_parity();
  } else {
    clear_has_parity();
  }
  parity_.SetAllocatedNoArena(&::rrts::driver::serial::SerialPortConfig::_default_parity_.get(), parity);
  // @@protoc_insertion_point(field_set_allocated:rrts.driver.serial.SerialPortConfig.parity)
}

// optional bool is_debug = 7 [default = true];
inline bool SerialPortConfig::has_is_debug() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void SerialPortConfig::set_has_is_debug() {
  _has_bits_[0] |= 0x00000010u;
}
inline void SerialPortConfig::clear_has_is_debug() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void SerialPortConfig::clear_is_debug() {
  is_debug_ = true;
  clear_has_is_debug();
}
inline bool SerialPortConfig::is_debug() const {
  // @@protoc_insertion_point(field_get:rrts.driver.serial.SerialPortConfig.is_debug)
  return is_debug_;
}
inline void SerialPortConfig::set_is_debug(bool value) {
  set_has_is_debug();
  is_debug_ = value;
  // @@protoc_insertion_point(field_set:rrts.driver.serial.SerialPortConfig.is_debug)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace serial
}  // namespace driver
}  // namespace rrts

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_serial_5fcom_5fconfig_2eproto_INCLUDED
