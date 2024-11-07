/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: network_constants.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif

#include "network_constants.pb-c.h"
void   wifi_connected_state__init
                     (WifiConnectedState         *message)
{
  static const WifiConnectedState init_value = WIFI_CONNECTED_STATE__INIT;
  *message = init_value;
}
size_t wifi_connected_state__get_packed_size
                     (const WifiConnectedState *message)
{
  assert(message->base.descriptor == &wifi_connected_state__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t wifi_connected_state__pack
                     (const WifiConnectedState *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &wifi_connected_state__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t wifi_connected_state__pack_to_buffer
                     (const WifiConnectedState *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &wifi_connected_state__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
WifiConnectedState *
       wifi_connected_state__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (WifiConnectedState *)
     protobuf_c_message_unpack (&wifi_connected_state__descriptor,
                                allocator, len, data);
}
void   wifi_connected_state__free_unpacked
                     (WifiConnectedState *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &wifi_connected_state__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
void   thread_attach_state__init
                     (ThreadAttachState         *message)
{
  static const ThreadAttachState init_value = THREAD_ATTACH_STATE__INIT;
  *message = init_value;
}
size_t thread_attach_state__get_packed_size
                     (const ThreadAttachState *message)
{
  assert(message->base.descriptor == &thread_attach_state__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t thread_attach_state__pack
                     (const ThreadAttachState *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &thread_attach_state__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t thread_attach_state__pack_to_buffer
                     (const ThreadAttachState *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &thread_attach_state__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
ThreadAttachState *
       thread_attach_state__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (ThreadAttachState *)
     protobuf_c_message_unpack (&thread_attach_state__descriptor,
                                allocator, len, data);
}
void   thread_attach_state__free_unpacked
                     (ThreadAttachState *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &thread_attach_state__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
static const ProtobufCFieldDescriptor wifi_connected_state__field_descriptors[5] =
{
  {
    "ip4_addr",
    1,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_STRING,
    0,   /* quantifier_offset */
    offsetof(WifiConnectedState, ip4_addr),
    NULL,
    &protobuf_c_empty_string,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "auth_mode",
    2,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_ENUM,
    0,   /* quantifier_offset */
    offsetof(WifiConnectedState, auth_mode),
    &wifi_auth_mode__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "ssid",
    3,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_BYTES,
    0,   /* quantifier_offset */
    offsetof(WifiConnectedState, ssid),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "bssid",
    4,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_BYTES,
    0,   /* quantifier_offset */
    offsetof(WifiConnectedState, bssid),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "channel",
    5,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_INT32,
    0,   /* quantifier_offset */
    offsetof(WifiConnectedState, channel),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned wifi_connected_state__field_indices_by_name[] = {
  1,   /* field[1] = auth_mode */
  3,   /* field[3] = bssid */
  4,   /* field[4] = channel */
  0,   /* field[0] = ip4_addr */
  2,   /* field[2] = ssid */
};
static const ProtobufCIntRange wifi_connected_state__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 5 }
};
const ProtobufCMessageDescriptor wifi_connected_state__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "WifiConnectedState",
  "WifiConnectedState",
  "WifiConnectedState",
  "",
  sizeof(WifiConnectedState),
  5,
  wifi_connected_state__field_descriptors,
  wifi_connected_state__field_indices_by_name,
  1,  wifi_connected_state__number_ranges,
  (ProtobufCMessageInit) wifi_connected_state__init,
  NULL,NULL,NULL    /* reserved[123] */
};
static const ProtobufCFieldDescriptor thread_attach_state__field_descriptors[4] =
{
  {
    "pan_id",
    1,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_UINT32,
    0,   /* quantifier_offset */
    offsetof(ThreadAttachState, pan_id),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "ext_pan_id",
    2,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_BYTES,
    0,   /* quantifier_offset */
    offsetof(ThreadAttachState, ext_pan_id),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "channel",
    3,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_UINT32,
    0,   /* quantifier_offset */
    offsetof(ThreadAttachState, channel),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "name",
    4,
    PROTOBUF_C_LABEL_NONE,
    PROTOBUF_C_TYPE_STRING,
    0,   /* quantifier_offset */
    offsetof(ThreadAttachState, name),
    NULL,
    &protobuf_c_empty_string,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned thread_attach_state__field_indices_by_name[] = {
  2,   /* field[2] = channel */
  1,   /* field[1] = ext_pan_id */
  3,   /* field[3] = name */
  0,   /* field[0] = pan_id */
};
static const ProtobufCIntRange thread_attach_state__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 4 }
};
const ProtobufCMessageDescriptor thread_attach_state__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "ThreadAttachState",
  "ThreadAttachState",
  "ThreadAttachState",
  "",
  sizeof(ThreadAttachState),
  4,
  thread_attach_state__field_descriptors,
  thread_attach_state__field_indices_by_name,
  1,  thread_attach_state__number_ranges,
  (ProtobufCMessageInit) thread_attach_state__init,
  NULL,NULL,NULL    /* reserved[123] */
};
static const ProtobufCEnumValue wifi_station_state__enum_values_by_number[4] =
{
  { "Connected", "WIFI_STATION_STATE__Connected", 0 },
  { "Connecting", "WIFI_STATION_STATE__Connecting", 1 },
  { "Disconnected", "WIFI_STATION_STATE__Disconnected", 2 },
  { "ConnectionFailed", "WIFI_STATION_STATE__ConnectionFailed", 3 },
};
static const ProtobufCIntRange wifi_station_state__value_ranges[] = {
{0, 0},{0, 4}
};
static const ProtobufCEnumValueIndex wifi_station_state__enum_values_by_name[4] =
{
  { "Connected", 0 },
  { "Connecting", 1 },
  { "ConnectionFailed", 3 },
  { "Disconnected", 2 },
};
const ProtobufCEnumDescriptor wifi_station_state__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "WifiStationState",
  "WifiStationState",
  "WifiStationState",
  "",
  4,
  wifi_station_state__enum_values_by_number,
  4,
  wifi_station_state__enum_values_by_name,
  1,
  wifi_station_state__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
static const ProtobufCEnumValue wifi_connect_failed_reason__enum_values_by_number[2] =
{
  { "AuthError", "WIFI_CONNECT_FAILED_REASON__AuthError", 0 },
  { "WifiNetworkNotFound", "WIFI_CONNECT_FAILED_REASON__WifiNetworkNotFound", 1 },
};
static const ProtobufCIntRange wifi_connect_failed_reason__value_ranges[] = {
{0, 0},{0, 2}
};
static const ProtobufCEnumValueIndex wifi_connect_failed_reason__enum_values_by_name[2] =
{
  { "AuthError", 0 },
  { "WifiNetworkNotFound", 1 },
};
const ProtobufCEnumDescriptor wifi_connect_failed_reason__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "WifiConnectFailedReason",
  "WifiConnectFailedReason",
  "WifiConnectFailedReason",
  "",
  2,
  wifi_connect_failed_reason__enum_values_by_number,
  2,
  wifi_connect_failed_reason__enum_values_by_name,
  1,
  wifi_connect_failed_reason__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
static const ProtobufCEnumValue wifi_auth_mode__enum_values_by_number[8] =
{
  { "Open", "WIFI_AUTH_MODE__Open", 0 },
  { "WEP", "WIFI_AUTH_MODE__WEP", 1 },
  { "WPA_PSK", "WIFI_AUTH_MODE__WPA_PSK", 2 },
  { "WPA2_PSK", "WIFI_AUTH_MODE__WPA2_PSK", 3 },
  { "WPA_WPA2_PSK", "WIFI_AUTH_MODE__WPA_WPA2_PSK", 4 },
  { "WPA2_ENTERPRISE", "WIFI_AUTH_MODE__WPA2_ENTERPRISE", 5 },
  { "WPA3_PSK", "WIFI_AUTH_MODE__WPA3_PSK", 6 },
  { "WPA2_WPA3_PSK", "WIFI_AUTH_MODE__WPA2_WPA3_PSK", 7 },
};
static const ProtobufCIntRange wifi_auth_mode__value_ranges[] = {
{0, 0},{0, 8}
};
static const ProtobufCEnumValueIndex wifi_auth_mode__enum_values_by_name[8] =
{
  { "Open", 0 },
  { "WEP", 1 },
  { "WPA2_ENTERPRISE", 5 },
  { "WPA2_PSK", 3 },
  { "WPA2_WPA3_PSK", 7 },
  { "WPA3_PSK", 6 },
  { "WPA_PSK", 2 },
  { "WPA_WPA2_PSK", 4 },
};
const ProtobufCEnumDescriptor wifi_auth_mode__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "WifiAuthMode",
  "WifiAuthMode",
  "WifiAuthMode",
  "",
  8,
  wifi_auth_mode__enum_values_by_number,
  8,
  wifi_auth_mode__enum_values_by_name,
  1,
  wifi_auth_mode__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
static const ProtobufCEnumValue thread_network_state__enum_values_by_number[4] =
{
  { "Attached", "THREAD_NETWORK_STATE__Attached", 0 },
  { "Attaching", "THREAD_NETWORK_STATE__Attaching", 1 },
  { "Dettached", "THREAD_NETWORK_STATE__Dettached", 2 },
  { "AttachingFailed", "THREAD_NETWORK_STATE__AttachingFailed", 3 },
};
static const ProtobufCIntRange thread_network_state__value_ranges[] = {
{0, 0},{0, 4}
};
static const ProtobufCEnumValueIndex thread_network_state__enum_values_by_name[4] =
{
  { "Attached", 0 },
  { "Attaching", 1 },
  { "AttachingFailed", 3 },
  { "Dettached", 2 },
};
const ProtobufCEnumDescriptor thread_network_state__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "ThreadNetworkState",
  "ThreadNetworkState",
  "ThreadNetworkState",
  "",
  4,
  thread_network_state__enum_values_by_number,
  4,
  thread_network_state__enum_values_by_name,
  1,
  thread_network_state__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};
static const ProtobufCEnumValue thread_attach_failed_reason__enum_values_by_number[2] =
{
  { "DatasetInvalid", "THREAD_ATTACH_FAILED_REASON__DatasetInvalid", 0 },
  { "ThreadNetworkNotFound", "THREAD_ATTACH_FAILED_REASON__ThreadNetworkNotFound", 1 },
};
static const ProtobufCIntRange thread_attach_failed_reason__value_ranges[] = {
{0, 0},{0, 2}
};
static const ProtobufCEnumValueIndex thread_attach_failed_reason__enum_values_by_name[2] =
{
  { "DatasetInvalid", 0 },
  { "ThreadNetworkNotFound", 1 },
};
const ProtobufCEnumDescriptor thread_attach_failed_reason__descriptor =
{
  PROTOBUF_C__ENUM_DESCRIPTOR_MAGIC,
  "ThreadAttachFailedReason",
  "ThreadAttachFailedReason",
  "ThreadAttachFailedReason",
  "",
  2,
  thread_attach_failed_reason__enum_values_by_number,
  2,
  thread_attach_failed_reason__enum_values_by_name,
  1,
  thread_attach_failed_reason__value_ranges,
  NULL,NULL,NULL,NULL   /* reserved[1234] */
};