#pragma once
// MESSAGE UAVCAN_NETWORK_STATISTICS PACKING

#define MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS 6

MAVPACKED(
typedef struct __mavlink_uavcan_network_statistics_t {
 uint32_t rx_msgs; /*< Nr of received can messages of master slave*/
 uint32_t tx_msgs; /*< Nr of transmitted can messages of master slave*/
 uint32_t errors; /*< Nr of observed CAN errors*/
}) mavlink_uavcan_network_statistics_t;

#define MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN 12
#define MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_MIN_LEN 12
#define MAVLINK_MSG_ID_6_LEN 12
#define MAVLINK_MSG_ID_6_MIN_LEN 12

#define MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_CRC 94
#define MAVLINK_MSG_ID_6_CRC 94



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAVCAN_NETWORK_STATISTICS { \
    6, \
    "UAVCAN_NETWORK_STATISTICS", \
    3, \
    {  { "rx_msgs", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_uavcan_network_statistics_t, rx_msgs) }, \
         { "tx_msgs", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_uavcan_network_statistics_t, tx_msgs) }, \
         { "errors", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_uavcan_network_statistics_t, errors) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAVCAN_NETWORK_STATISTICS { \
    "UAVCAN_NETWORK_STATISTICS", \
    3, \
    {  { "rx_msgs", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_uavcan_network_statistics_t, rx_msgs) }, \
         { "tx_msgs", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_uavcan_network_statistics_t, tx_msgs) }, \
         { "errors", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_uavcan_network_statistics_t, errors) }, \
         } \
}
#endif

/**
 * @brief Pack a uavcan_network_statistics message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rx_msgs Nr of received can messages of master slave
 * @param tx_msgs Nr of transmitted can messages of master slave
 * @param errors Nr of observed CAN errors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavcan_network_statistics_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t rx_msgs, uint32_t tx_msgs, uint32_t errors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN];
    _mav_put_uint32_t(buf, 0, rx_msgs);
    _mav_put_uint32_t(buf, 4, tx_msgs);
    _mav_put_uint32_t(buf, 8, errors);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN);
#else
    mavlink_uavcan_network_statistics_t packet;
    packet.rx_msgs = rx_msgs;
    packet.tx_msgs = tx_msgs;
    packet.errors = errors;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_CRC);
}

/**
 * @brief Pack a uavcan_network_statistics message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rx_msgs Nr of received can messages of master slave
 * @param tx_msgs Nr of transmitted can messages of master slave
 * @param errors Nr of observed CAN errors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavcan_network_statistics_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t rx_msgs,uint32_t tx_msgs,uint32_t errors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN];
    _mav_put_uint32_t(buf, 0, rx_msgs);
    _mav_put_uint32_t(buf, 4, tx_msgs);
    _mav_put_uint32_t(buf, 8, errors);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN);
#else
    mavlink_uavcan_network_statistics_t packet;
    packet.rx_msgs = rx_msgs;
    packet.tx_msgs = tx_msgs;
    packet.errors = errors;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_CRC);
}

/**
 * @brief Encode a uavcan_network_statistics struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uavcan_network_statistics C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavcan_network_statistics_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uavcan_network_statistics_t* uavcan_network_statistics)
{
    return mavlink_msg_uavcan_network_statistics_pack(system_id, component_id, msg, uavcan_network_statistics->rx_msgs, uavcan_network_statistics->tx_msgs, uavcan_network_statistics->errors);
}

/**
 * @brief Encode a uavcan_network_statistics struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uavcan_network_statistics C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavcan_network_statistics_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uavcan_network_statistics_t* uavcan_network_statistics)
{
    return mavlink_msg_uavcan_network_statistics_pack_chan(system_id, component_id, chan, msg, uavcan_network_statistics->rx_msgs, uavcan_network_statistics->tx_msgs, uavcan_network_statistics->errors);
}

/**
 * @brief Send a uavcan_network_statistics message
 * @param chan MAVLink channel to send the message
 *
 * @param rx_msgs Nr of received can messages of master slave
 * @param tx_msgs Nr of transmitted can messages of master slave
 * @param errors Nr of observed CAN errors
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uavcan_network_statistics_send(mavlink_channel_t chan, uint32_t rx_msgs, uint32_t tx_msgs, uint32_t errors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN];
    _mav_put_uint32_t(buf, 0, rx_msgs);
    _mav_put_uint32_t(buf, 4, tx_msgs);
    _mav_put_uint32_t(buf, 8, errors);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS, buf, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_CRC);
#else
    mavlink_uavcan_network_statistics_t packet;
    packet.rx_msgs = rx_msgs;
    packet.tx_msgs = tx_msgs;
    packet.errors = errors;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS, (const char *)&packet, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_CRC);
#endif
}

/**
 * @brief Send a uavcan_network_statistics message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uavcan_network_statistics_send_struct(mavlink_channel_t chan, const mavlink_uavcan_network_statistics_t* uavcan_network_statistics)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavcan_network_statistics_send(chan, uavcan_network_statistics->rx_msgs, uavcan_network_statistics->tx_msgs, uavcan_network_statistics->errors);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS, (const char *)uavcan_network_statistics, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uavcan_network_statistics_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t rx_msgs, uint32_t tx_msgs, uint32_t errors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, rx_msgs);
    _mav_put_uint32_t(buf, 4, tx_msgs);
    _mav_put_uint32_t(buf, 8, errors);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS, buf, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_CRC);
#else
    mavlink_uavcan_network_statistics_t *packet = (mavlink_uavcan_network_statistics_t *)msgbuf;
    packet->rx_msgs = rx_msgs;
    packet->tx_msgs = tx_msgs;
    packet->errors = errors;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS, (const char *)packet, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_CRC);
#endif
}
#endif

#endif

// MESSAGE UAVCAN_NETWORK_STATISTICS UNPACKING


/**
 * @brief Get field rx_msgs from uavcan_network_statistics message
 *
 * @return Nr of received can messages of master slave
 */
static inline uint32_t mavlink_msg_uavcan_network_statistics_get_rx_msgs(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field tx_msgs from uavcan_network_statistics message
 *
 * @return Nr of transmitted can messages of master slave
 */
static inline uint32_t mavlink_msg_uavcan_network_statistics_get_tx_msgs(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field errors from uavcan_network_statistics message
 *
 * @return Nr of observed CAN errors
 */
static inline uint32_t mavlink_msg_uavcan_network_statistics_get_errors(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Decode a uavcan_network_statistics message into a struct
 *
 * @param msg The message to decode
 * @param uavcan_network_statistics C-struct to decode the message contents into
 */
static inline void mavlink_msg_uavcan_network_statistics_decode(const mavlink_message_t* msg, mavlink_uavcan_network_statistics_t* uavcan_network_statistics)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uavcan_network_statistics->rx_msgs = mavlink_msg_uavcan_network_statistics_get_rx_msgs(msg);
    uavcan_network_statistics->tx_msgs = mavlink_msg_uavcan_network_statistics_get_tx_msgs(msg);
    uavcan_network_statistics->errors = mavlink_msg_uavcan_network_statistics_get_errors(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN? msg->len : MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN;
        memset(uavcan_network_statistics, 0, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_LEN);
    memcpy(uavcan_network_statistics, _MAV_PAYLOAD(msg), len);
#endif
}
