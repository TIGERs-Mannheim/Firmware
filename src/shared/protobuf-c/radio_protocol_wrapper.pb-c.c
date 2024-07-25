/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: radio_protocol_wrapper.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif

#include "radio_protocol_wrapper.pb-c.h"
void   radio_protocol_wrapper__init
                     (RadioProtocolWrapper         *message)
{
  static RadioProtocolWrapper init_value = RADIO_PROTOCOL_WRAPPER__INIT;
  *message = init_value;
}
size_t radio_protocol_wrapper__get_packed_size
                     (const RadioProtocolWrapper *message)
{
  assert(message->base.descriptor == &radio_protocol_wrapper__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t radio_protocol_wrapper__pack
                     (const RadioProtocolWrapper *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &radio_protocol_wrapper__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t radio_protocol_wrapper__pack_to_buffer
                     (const RadioProtocolWrapper *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &radio_protocol_wrapper__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
RadioProtocolWrapper *
       radio_protocol_wrapper__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (RadioProtocolWrapper *)
     protobuf_c_message_unpack (&radio_protocol_wrapper__descriptor,
                                allocator, len, data);
}
void   radio_protocol_wrapper__free_unpacked
                     (RadioProtocolWrapper *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &radio_protocol_wrapper__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
static const ProtobufCFieldDescriptor radio_protocol_wrapper__field_descriptors[1] =
{
  {
    "command",
    1,
    PROTOBUF_C_LABEL_REPEATED,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(RadioProtocolWrapper, n_command),
    offsetof(RadioProtocolWrapper, command),
    &radio_protocol_command__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned radio_protocol_wrapper__field_indices_by_name[] = {
  0,   /* field[0] = command */
};
static const ProtobufCIntRange radio_protocol_wrapper__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 1 }
};
const ProtobufCMessageDescriptor radio_protocol_wrapper__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "RadioProtocolWrapper",
  "RadioProtocolWrapper",
  "RadioProtocolWrapper",
  "",
  sizeof(RadioProtocolWrapper),
  1,
  radio_protocol_wrapper__field_descriptors,
  radio_protocol_wrapper__field_indices_by_name,
  1,  radio_protocol_wrapper__number_ranges,
  (ProtobufCMessageInit) radio_protocol_wrapper__init,
  NULL,NULL,NULL    /* reserved[123] */
};