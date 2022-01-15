// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: armor_detection.proto

#include "armor_detection.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

namespace armor_detection {
class ArmorDetectionParamDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<ArmorDetectionParam>
      _instance;
} _ArmorDetectionParam_default_instance_;
}  // namespace armor_detection
namespace protobuf_armor_5fdetection_2eproto {
static void InitDefaultsArmorDetectionParam() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::armor_detection::_ArmorDetectionParam_default_instance_;
    new (ptr) ::armor_detection::ArmorDetectionParam();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::armor_detection::ArmorDetectionParam::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_ArmorDetectionParam =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsArmorDetectionParam}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_ArmorDetectionParam.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::armor_detection::ArmorDetectionParam, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::armor_detection::ArmorDetectionParam, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::armor_detection::ArmorDetectionParam, undetected_armor_delay_),
  0,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 6, sizeof(::armor_detection::ArmorDetectionParam)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::armor_detection::_ArmorDetectionParam_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "armor_detection.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\025armor_detection.proto\022\017armor_detection"
      "\"5\n\023ArmorDetectionParam\022\036\n\026undetected_ar"
      "mor_delay\030\001 \001(\r"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 95);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "armor_detection.proto", &protobuf_RegisterTypes);
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_armor_5fdetection_2eproto
namespace armor_detection {

// ===================================================================

void ArmorDetectionParam::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int ArmorDetectionParam::kUndetectedArmorDelayFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

ArmorDetectionParam::ArmorDetectionParam()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_armor_5fdetection_2eproto::scc_info_ArmorDetectionParam.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:armor_detection.ArmorDetectionParam)
}
ArmorDetectionParam::ArmorDetectionParam(const ArmorDetectionParam& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  undetected_armor_delay_ = from.undetected_armor_delay_;
  // @@protoc_insertion_point(copy_constructor:armor_detection.ArmorDetectionParam)
}

void ArmorDetectionParam::SharedCtor() {
  undetected_armor_delay_ = 0u;
}

ArmorDetectionParam::~ArmorDetectionParam() {
  // @@protoc_insertion_point(destructor:armor_detection.ArmorDetectionParam)
  SharedDtor();
}

void ArmorDetectionParam::SharedDtor() {
}

void ArmorDetectionParam::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* ArmorDetectionParam::descriptor() {
  ::protobuf_armor_5fdetection_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_armor_5fdetection_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const ArmorDetectionParam& ArmorDetectionParam::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_armor_5fdetection_2eproto::scc_info_ArmorDetectionParam.base);
  return *internal_default_instance();
}


void ArmorDetectionParam::Clear() {
// @@protoc_insertion_point(message_clear_start:armor_detection.ArmorDetectionParam)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  undetected_armor_delay_ = 0u;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool ArmorDetectionParam::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:armor_detection.ArmorDetectionParam)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional uint32 undetected_armor_delay = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          set_has_undetected_armor_delay();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &undetected_armor_delay_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:armor_detection.ArmorDetectionParam)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:armor_detection.ArmorDetectionParam)
  return false;
#undef DO_
}

void ArmorDetectionParam::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:armor_detection.ArmorDetectionParam)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional uint32 undetected_armor_delay = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(1, this->undetected_armor_delay(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:armor_detection.ArmorDetectionParam)
}

::google::protobuf::uint8* ArmorDetectionParam::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:armor_detection.ArmorDetectionParam)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional uint32 undetected_armor_delay = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(1, this->undetected_armor_delay(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:armor_detection.ArmorDetectionParam)
  return target;
}

size_t ArmorDetectionParam::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:armor_detection.ArmorDetectionParam)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // optional uint32 undetected_armor_delay = 1;
  if (has_undetected_armor_delay()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::UInt32Size(
        this->undetected_armor_delay());
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ArmorDetectionParam::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:armor_detection.ArmorDetectionParam)
  GOOGLE_DCHECK_NE(&from, this);
  const ArmorDetectionParam* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const ArmorDetectionParam>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:armor_detection.ArmorDetectionParam)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:armor_detection.ArmorDetectionParam)
    MergeFrom(*source);
  }
}

void ArmorDetectionParam::MergeFrom(const ArmorDetectionParam& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:armor_detection.ArmorDetectionParam)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.has_undetected_armor_delay()) {
    set_undetected_armor_delay(from.undetected_armor_delay());
  }
}

void ArmorDetectionParam::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:armor_detection.ArmorDetectionParam)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ArmorDetectionParam::CopyFrom(const ArmorDetectionParam& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:armor_detection.ArmorDetectionParam)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ArmorDetectionParam::IsInitialized() const {
  return true;
}

void ArmorDetectionParam::Swap(ArmorDetectionParam* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ArmorDetectionParam::InternalSwap(ArmorDetectionParam* other) {
  using std::swap;
  swap(undetected_armor_delay_, other->undetected_armor_delay_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata ArmorDetectionParam::GetMetadata() const {
  protobuf_armor_5fdetection_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_armor_5fdetection_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace armor_detection
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::armor_detection::ArmorDetectionParam* Arena::CreateMaybeMessage< ::armor_detection::ArmorDetectionParam >(Arena* arena) {
  return Arena::CreateInternal< ::armor_detection::ArmorDetectionParam >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
