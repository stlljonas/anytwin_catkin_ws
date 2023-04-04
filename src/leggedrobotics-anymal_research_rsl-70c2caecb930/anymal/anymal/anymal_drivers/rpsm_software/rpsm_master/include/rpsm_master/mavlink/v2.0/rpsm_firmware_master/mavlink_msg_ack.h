#pragma once
// MESSAGE ACK PACKING

#define MAVLINK_MSG_ID_ACK 30

MAVPACKED(
typedef struct __mavlink_ack_t {
 uint8_t reason; /*< slave_id of which we would like to get data*/
}) mavlink_ack_t;

#define MAVLINK_MSG_ID_ACK_LEN 1
#define MAVLINK_MSG_ID_ACK_MIN_LEN 1
#define MAVLINK_MSG_ID_30_LEN 1
#define MAVLINK_MSG_ID_30_MIN_LEN 1

#define MAVLINK_MSG_ID_ACK_CRC 150
#define MAVLINK_MSG_ID_30_CRC 150



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACK { \
    30, \
    "ACK", \
    1, \
    {  { "reason", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_ack_t, reason) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACK { \
    "ACK", \
    1, \
    {  { "reason", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_ack_t, reason) }, \
         } \
}
#endif

/**
 * @brief Pack a ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param reason slave_id of which we would like to get data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACK_LEN];
    _mav_put_uint8_t(buf, 0, reason);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACK_LEN);
#else
    mavlink_ack_t packet;
    packet.reason = reason;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACK_MIN_LEN, MAVLINK_MSG_ID_ACK_LEN, MAVLINK_MSG_ID_ACK_CRC);
}

/**
 * @brief Pack a ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param reason slave_id of which we would like to get data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACK_LEN];
    _mav_put_uint8_t(buf, 0, reason);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACK_LEN);
#else
    mavlink_ack_t packet;
    packet.reason = reason;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACK_MIN_LEN, MAVLINK_MSG_ID_ACK_LEN, MAVLINK_MSG_ID_ACK_CRC);
}

/**
 * @brief Encode a ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ack_t* ack)
{
    return mavlink_msg_ack_pack(system_id, component_id, msg, ack->reason);
}

/**
 * @brief Encode a ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ack_t* ack)
{
    return mavlink_msg_ack_pack_chan(system_id, component_id, chan, msg, ack->reason);
}

/**
 * @brief Send a ack message
 * @param chan MAVLink channel to send the message
 *
 * @param reason slave_id of which we would like to get data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ack_send(mavlink_channel_t chan, uint8_t reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACK_LEN];
    _mav_put_uint8_t(buf, 0, reason);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACK, buf, MAVLINK_MSG_ID_ACK_MIN_LEN, MAVLINK_MSG_ID_ACK_LEN, MAVLINK_MSG_ID_ACK_CRC);
#else
    mavlink_ack_t packet;
    packet.reason = reason;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACK, (const char *)&packet, MAVLINK_MSG_ID_ACK_MIN_LEN, MAVLINK_MSG_ID_ACK_LEN, MAVLINK_MSG_ID_ACK_CRC);
#endif
}

/**
 * @brief Send a ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ack_send_struct(mavlink_channel_t chan, const mavlink_ack_t* ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ack_send(chan, ack->reason);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACK, (const char *)ack, MAVLINK_MSG_ID_ACK_MIN_LEN, MAVLINK_MSG_ID_ACK_LEN, MAVLINK_MSG_ID_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, reason);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACK, buf, MAVLINK_MSG_ID_ACK_MIN_LEN, MAVLINK_MSG_ID_ACK_LEN, MAVLINK_MSG_ID_ACK_CRC);
#else
    mavlink_ack_t *packet = (mavlink_ack_t *)msgbuf;
    packet->reason = reason;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACK, (const char *)packet, MAVLINK_MSG_ID_ACK_MIN_LEN, MAVLINK_MSG_ID_ACK_LEN, MAVLINK_MSG_ID_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE ACK UNPACKING


/**
 * @brief Get field reason from ack message
 *
 * @return slave_id of which we would like to get data
 */
static inline uint8_t mavlink_msg_ack_get_reason(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a ack message into a struct
 *
 * @param msg The message to decode
 * @param ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_ack_decode(const mavlink_message_t* msg, mavlink_ack_t* ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ack->reason = mavlink_msg_ack_get_reason(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACK_LEN? msg->len : MAVLINK_MSG_ID_ACK_LEN;
        memset(ack, 0, MAVLINK_MSG_ID_ACK_LEN);
    memcpy(ack, _MAV_PAYLOAD(msg), len);
#endif
}
