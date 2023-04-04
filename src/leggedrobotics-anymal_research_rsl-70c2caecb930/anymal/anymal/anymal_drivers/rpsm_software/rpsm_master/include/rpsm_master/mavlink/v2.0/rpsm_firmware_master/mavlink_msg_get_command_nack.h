#pragma once
// MESSAGE GET_COMMAND_NACK PACKING

#define MAVLINK_MSG_ID_GET_COMMAND_NACK 31

MAVPACKED(
typedef struct __mavlink_get_command_nack_t {
 uint8_t error_number; /*< why did it fail*/
}) mavlink_get_command_nack_t;

#define MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN 1
#define MAVLINK_MSG_ID_GET_COMMAND_NACK_MIN_LEN 1
#define MAVLINK_MSG_ID_31_LEN 1
#define MAVLINK_MSG_ID_31_MIN_LEN 1

#define MAVLINK_MSG_ID_GET_COMMAND_NACK_CRC 152
#define MAVLINK_MSG_ID_31_CRC 152



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GET_COMMAND_NACK { \
    31, \
    "GET_COMMAND_NACK", \
    1, \
    {  { "error_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_get_command_nack_t, error_number) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GET_COMMAND_NACK { \
    "GET_COMMAND_NACK", \
    1, \
    {  { "error_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_get_command_nack_t, error_number) }, \
         } \
}
#endif

/**
 * @brief Pack a get_command_nack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param error_number why did it fail
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_get_command_nack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t error_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN];
    _mav_put_uint8_t(buf, 0, error_number);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN);
#else
    mavlink_get_command_nack_t packet;
    packet.error_number = error_number;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GET_COMMAND_NACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GET_COMMAND_NACK_MIN_LEN, MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN, MAVLINK_MSG_ID_GET_COMMAND_NACK_CRC);
}

/**
 * @brief Pack a get_command_nack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param error_number why did it fail
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_get_command_nack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t error_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN];
    _mav_put_uint8_t(buf, 0, error_number);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN);
#else
    mavlink_get_command_nack_t packet;
    packet.error_number = error_number;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GET_COMMAND_NACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GET_COMMAND_NACK_MIN_LEN, MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN, MAVLINK_MSG_ID_GET_COMMAND_NACK_CRC);
}

/**
 * @brief Encode a get_command_nack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param get_command_nack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_get_command_nack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_get_command_nack_t* get_command_nack)
{
    return mavlink_msg_get_command_nack_pack(system_id, component_id, msg, get_command_nack->error_number);
}

/**
 * @brief Encode a get_command_nack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param get_command_nack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_get_command_nack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_get_command_nack_t* get_command_nack)
{
    return mavlink_msg_get_command_nack_pack_chan(system_id, component_id, chan, msg, get_command_nack->error_number);
}

/**
 * @brief Send a get_command_nack message
 * @param chan MAVLink channel to send the message
 *
 * @param error_number why did it fail
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_get_command_nack_send(mavlink_channel_t chan, uint8_t error_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN];
    _mav_put_uint8_t(buf, 0, error_number);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_COMMAND_NACK, buf, MAVLINK_MSG_ID_GET_COMMAND_NACK_MIN_LEN, MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN, MAVLINK_MSG_ID_GET_COMMAND_NACK_CRC);
#else
    mavlink_get_command_nack_t packet;
    packet.error_number = error_number;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_COMMAND_NACK, (const char *)&packet, MAVLINK_MSG_ID_GET_COMMAND_NACK_MIN_LEN, MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN, MAVLINK_MSG_ID_GET_COMMAND_NACK_CRC);
#endif
}

/**
 * @brief Send a get_command_nack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_get_command_nack_send_struct(mavlink_channel_t chan, const mavlink_get_command_nack_t* get_command_nack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_get_command_nack_send(chan, get_command_nack->error_number);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_COMMAND_NACK, (const char *)get_command_nack, MAVLINK_MSG_ID_GET_COMMAND_NACK_MIN_LEN, MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN, MAVLINK_MSG_ID_GET_COMMAND_NACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_get_command_nack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t error_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, error_number);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_COMMAND_NACK, buf, MAVLINK_MSG_ID_GET_COMMAND_NACK_MIN_LEN, MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN, MAVLINK_MSG_ID_GET_COMMAND_NACK_CRC);
#else
    mavlink_get_command_nack_t *packet = (mavlink_get_command_nack_t *)msgbuf;
    packet->error_number = error_number;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_COMMAND_NACK, (const char *)packet, MAVLINK_MSG_ID_GET_COMMAND_NACK_MIN_LEN, MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN, MAVLINK_MSG_ID_GET_COMMAND_NACK_CRC);
#endif
}
#endif

#endif

// MESSAGE GET_COMMAND_NACK UNPACKING


/**
 * @brief Get field error_number from get_command_nack message
 *
 * @return why did it fail
 */
static inline uint8_t mavlink_msg_get_command_nack_get_error_number(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a get_command_nack message into a struct
 *
 * @param msg The message to decode
 * @param get_command_nack C-struct to decode the message contents into
 */
static inline void mavlink_msg_get_command_nack_decode(const mavlink_message_t* msg, mavlink_get_command_nack_t* get_command_nack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    get_command_nack->error_number = mavlink_msg_get_command_nack_get_error_number(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN? msg->len : MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN;
        memset(get_command_nack, 0, MAVLINK_MSG_ID_GET_COMMAND_NACK_LEN);
    memcpy(get_command_nack, _MAV_PAYLOAD(msg), len);
#endif
}
