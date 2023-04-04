#pragma once
// MESSAGE GET_COMMAND PACKING

#define MAVLINK_MSG_ID_GET_COMMAND 20

MAVPACKED(
typedef struct __mavlink_get_command_t {
 uint16_t get_type; /*< the Type of data we'd like to get, refer to
               available message types. The Server will try to respond with a message of the requested type.
               depending on what the type is, slave_id will be respected or not.*/
 uint8_t slave_id; /*< slave_id of which we would like to get data*/
}) mavlink_get_command_t;

#define MAVLINK_MSG_ID_GET_COMMAND_LEN 3
#define MAVLINK_MSG_ID_GET_COMMAND_MIN_LEN 3
#define MAVLINK_MSG_ID_20_LEN 3
#define MAVLINK_MSG_ID_20_MIN_LEN 3

#define MAVLINK_MSG_ID_GET_COMMAND_CRC 136
#define MAVLINK_MSG_ID_20_CRC 136



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GET_COMMAND { \
    20, \
    "GET_COMMAND", \
    2, \
    {  { "get_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_get_command_t, get_type) }, \
         { "slave_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_get_command_t, slave_id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GET_COMMAND { \
    "GET_COMMAND", \
    2, \
    {  { "get_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_get_command_t, get_type) }, \
         { "slave_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_get_command_t, slave_id) }, \
         } \
}
#endif

/**
 * @brief Pack a get_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param slave_id slave_id of which we would like to get data
 * @param get_type the Type of data we'd like to get, refer to
               available message types. The Server will try to respond with a message of the requested type.
               depending on what the type is, slave_id will be respected or not.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_get_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t slave_id, uint16_t get_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GET_COMMAND_LEN];
    _mav_put_uint16_t(buf, 0, get_type);
    _mav_put_uint8_t(buf, 2, slave_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GET_COMMAND_LEN);
#else
    mavlink_get_command_t packet;
    packet.get_type = get_type;
    packet.slave_id = slave_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GET_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GET_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GET_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GET_COMMAND_LEN, MAVLINK_MSG_ID_GET_COMMAND_CRC);
}

/**
 * @brief Pack a get_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param slave_id slave_id of which we would like to get data
 * @param get_type the Type of data we'd like to get, refer to
               available message types. The Server will try to respond with a message of the requested type.
               depending on what the type is, slave_id will be respected or not.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_get_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t slave_id,uint16_t get_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GET_COMMAND_LEN];
    _mav_put_uint16_t(buf, 0, get_type);
    _mav_put_uint8_t(buf, 2, slave_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GET_COMMAND_LEN);
#else
    mavlink_get_command_t packet;
    packet.get_type = get_type;
    packet.slave_id = slave_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GET_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GET_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GET_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GET_COMMAND_LEN, MAVLINK_MSG_ID_GET_COMMAND_CRC);
}

/**
 * @brief Encode a get_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param get_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_get_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_get_command_t* get_command)
{
    return mavlink_msg_get_command_pack(system_id, component_id, msg, get_command->slave_id, get_command->get_type);
}

/**
 * @brief Encode a get_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param get_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_get_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_get_command_t* get_command)
{
    return mavlink_msg_get_command_pack_chan(system_id, component_id, chan, msg, get_command->slave_id, get_command->get_type);
}

/**
 * @brief Send a get_command message
 * @param chan MAVLink channel to send the message
 *
 * @param slave_id slave_id of which we would like to get data
 * @param get_type the Type of data we'd like to get, refer to
               available message types. The Server will try to respond with a message of the requested type.
               depending on what the type is, slave_id will be respected or not.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_get_command_send(mavlink_channel_t chan, uint8_t slave_id, uint16_t get_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GET_COMMAND_LEN];
    _mav_put_uint16_t(buf, 0, get_type);
    _mav_put_uint8_t(buf, 2, slave_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_COMMAND, buf, MAVLINK_MSG_ID_GET_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GET_COMMAND_LEN, MAVLINK_MSG_ID_GET_COMMAND_CRC);
#else
    mavlink_get_command_t packet;
    packet.get_type = get_type;
    packet.slave_id = slave_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_GET_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GET_COMMAND_LEN, MAVLINK_MSG_ID_GET_COMMAND_CRC);
#endif
}

/**
 * @brief Send a get_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_get_command_send_struct(mavlink_channel_t chan, const mavlink_get_command_t* get_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_get_command_send(chan, get_command->slave_id, get_command->get_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_COMMAND, (const char *)get_command, MAVLINK_MSG_ID_GET_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GET_COMMAND_LEN, MAVLINK_MSG_ID_GET_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_GET_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_get_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t slave_id, uint16_t get_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, get_type);
    _mav_put_uint8_t(buf, 2, slave_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_COMMAND, buf, MAVLINK_MSG_ID_GET_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GET_COMMAND_LEN, MAVLINK_MSG_ID_GET_COMMAND_CRC);
#else
    mavlink_get_command_t *packet = (mavlink_get_command_t *)msgbuf;
    packet->get_type = get_type;
    packet->slave_id = slave_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_COMMAND, (const char *)packet, MAVLINK_MSG_ID_GET_COMMAND_MIN_LEN, MAVLINK_MSG_ID_GET_COMMAND_LEN, MAVLINK_MSG_ID_GET_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE GET_COMMAND UNPACKING


/**
 * @brief Get field slave_id from get_command message
 *
 * @return slave_id of which we would like to get data
 */
static inline uint8_t mavlink_msg_get_command_get_slave_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field get_type from get_command message
 *
 * @return the Type of data we'd like to get, refer to
               available message types. The Server will try to respond with a message of the requested type.
               depending on what the type is, slave_id will be respected or not.
 */
static inline uint16_t mavlink_msg_get_command_get_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a get_command message into a struct
 *
 * @param msg The message to decode
 * @param get_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_get_command_decode(const mavlink_message_t* msg, mavlink_get_command_t* get_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    get_command->get_type = mavlink_msg_get_command_get_get_type(msg);
    get_command->slave_id = mavlink_msg_get_command_get_slave_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GET_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_GET_COMMAND_LEN;
        memset(get_command, 0, MAVLINK_MSG_ID_GET_COMMAND_LEN);
    memcpy(get_command, _MAV_PAYLOAD(msg), len);
#endif
}
