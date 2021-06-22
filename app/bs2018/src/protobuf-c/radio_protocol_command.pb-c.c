/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: radio_protocol_command.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif

#include "radio_protocol_command.pb-c.h"
void   radio_protocol_command__init
                     (RadioProtocolCommand         *message)
{
  static RadioProtocolCommand init_value = RADIO_PROTOCOL_COMMAND__INIT;
  *message = init_value;
}
size_t radio_protocol_command__get_packed_size
                     (const RadioProtocolCommand *message)
{
  assert(message->base.descriptor == &radio_protocol_command__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t radio_protocol_command__pack
                     (const RadioProtocolCommand *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &radio_protocol_command__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t radio_protocol_command__pack_to_buffer
                     (const RadioProtocolCommand *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &radio_protocol_command__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
RadioProtocolCommand *
       radio_protocol_command__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (RadioProtocolCommand *)
     protobuf_c_message_unpack (&radio_protocol_command__descriptor,
                                allocator, len, data);
}
void   radio_protocol_command__free_unpacked
                     (RadioProtocolCommand *message,
                      ProtobufCAllocator *allocator)
{
  assert(message->base.descriptor == &radio_protocol_command__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
static const ProtobufCFieldDescriptor radio_protocol_command__field_descriptors[7] =
{
  {
    "robot_id",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_INT32,
    0,   /* quantifier_offset */
    offsetof(RadioProtocolCommand, robot_id),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "velocity_x",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(RadioProtocolCommand, velocity_x),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "velocity_y",
    3,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(RadioProtocolCommand, velocity_y),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "velocity_r",
    4,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_FLOAT,
    0,   /* quantifier_offset */
    offsetof(RadioProtocolCommand, velocity_r),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "flat_kick",
    5,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_FLOAT,
    offsetof(RadioProtocolCommand, has_flat_kick),
    offsetof(RadioProtocolCommand, flat_kick),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "chip_kick",
    6,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_FLOAT,
    offsetof(RadioProtocolCommand, has_chip_kick),
    offsetof(RadioProtocolCommand, chip_kick),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "dribbler_spin",
    7,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_FLOAT,
    offsetof(RadioProtocolCommand, has_dribbler_spin),
    offsetof(RadioProtocolCommand, dribbler_spin),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned radio_protocol_command__field_indices_by_name[] = {
  5,   /* field[5] = chip_kick */
  6,   /* field[6] = dribbler_spin */
  4,   /* field[4] = flat_kick */
  0,   /* field[0] = robot_id */
  3,   /* field[3] = velocity_r */
  1,   /* field[1] = velocity_x */
  2,   /* field[2] = velocity_y */
};
static const ProtobufCIntRange radio_protocol_command__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 7 }
};
const ProtobufCMessageDescriptor radio_protocol_command__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "RadioProtocolCommand",
  "RadioProtocolCommand",
  "RadioProtocolCommand",
  "",
  sizeof(RadioProtocolCommand),
  7,
  radio_protocol_command__field_descriptors,
  radio_protocol_command__field_indices_by_name,
  1,  radio_protocol_command__number_ranges,
  (ProtobufCMessageInit) radio_protocol_command__init,
  NULL,NULL,NULL    /* reserved[123] */
};
