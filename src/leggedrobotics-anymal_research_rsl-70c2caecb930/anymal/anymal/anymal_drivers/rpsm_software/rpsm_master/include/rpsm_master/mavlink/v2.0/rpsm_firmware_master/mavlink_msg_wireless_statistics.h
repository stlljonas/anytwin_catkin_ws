#pragma once
// MESSAGE WIRELESS_STATISTICS PACKING

#define MAVLINK_MSG_ID_WIRELESS_STATISTICS 7

MAVPACKED(
typedef struct __mavlink_wireless_statistics_t {
 uint32_t rx_bytes; /*< Nr of received bytes*/
 uint32_t tx_bytes; /*< Nr of sent bytes*/
 uint16_t rx_msgs; /*< Nr of received wireless messages of pwrmaster*/
 uint16_t rx_errors; /*< Nr of transmitted wireless messages of pwrmaster*/
}) mavlink_wireless_statistics_t;

#define MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN 12
#define MAVLINK_MSG_ID_WIRELESS_STATISTICS_MIN_LEN 12
#define MAVLINK_MSG_ID_7_LEN 12
#define MAVLINK_MSG_ID_7_MIN_LEN 12

#define MAVLINK_MSG_ID_WIRELESS_STATISTICS_CRC 190
#define MAVLINK_MSG_ID_7_CRC 190



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WIRELESS_STATISTICS { \
    7, \
    "WIRELESS_STATISTICS", \
    4, \
    {  { "rx_bytes", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_wireless_statistics_t, rx_bytes) }, \
         { "tx_bytes", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_wireless_statistics_t, tx_bytes) }, \
         { "rx_msgs", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_wireless_statistics_t, rx_msgs) }, \
         { "rx_errors", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_wireless_statistics_t, rx_errors) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WIRELESS_STATISTICS { \
    "WIRELESS_STATISTICS", \
    4, \
    {  { "rx_bytes", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_wireless_statistics_t, rx_bytes) }, \
         { "tx_bytes", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_wireless_statistics_t, tx_bytes) }, \
         { "rx_msgs", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_wireless_statistics_t, rx_msgs) }, \
         { "rx_errors", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_wireless_statistics_t, rx_errors) }, \
         } \
}
#endif

/**
 * @brief Pack a wireless_statistics message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rx_bytes Nr of received bytes
 * @param tx_bytes Nr of sent bytes
 * @param rx_msgs Nr of received wireless messages of pwrmaster
 * @param rx_errors Nr of transmitted wireless messages of pwrmaster
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wireless_statistics_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t rx_bytes, uint32_t tx_bytes, uint16_t rx_msgs, uint16_t rx_errors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN];
    _mav_put_uint32_t(buf, 0, rx_bytes);
    _mav_put_uint32_t(buf, 4, tx_bytes);
    _mav_put_uint16_t(buf, 8, rx_msgs);
    _mav_put_uint16_t(buf, 10, rx_errors);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN);
#else
    mavlink_wireless_statistics_t packet;
    packet.rx_bytes = rx_bytes;
    packet.tx_bytes = tx_bytes;
    packet.rx_msgs = rx_msgs;
    packet.rx_errors = rx_errors;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WIRELESS_STATISTICS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WIRELESS_STATISTICS_MIN_LEN, MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN, MAVLINK_MSG_ID_WIRELESS_STATISTICS_CRC);
}

/**
 * @brief Pack a wireless_statistics message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rx_bytes Nr of received bytes
 * @param tx_bytes Nr of sent bytes
 * @param rx_msgs Nr of received wireless messages of pwrmaster
 * @param rx_errors Nr of transmitted wireless messages of pwrmaster
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wireless_statistics_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t rx_bytes,uint32_t tx_bytes,uint16_t rx_msgs,uint16_t rx_errors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN];
    _mav_put_uint32_t(buf, 0, rx_bytes);
    _mav_put_uint32_t(buf, 4, tx_bytes);
    _mav_put_uint16_t(buf, 8, rx_msgs);
    _mav_put_uint16_t(buf, 10, rx_errors);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN);
#else
    mavlink_wireless_statistics_t packet;
    packet.rx_bytes = rx_bytes;
    packet.tx_bytes = tx_bytes;
    packet.rx_msgs = rx_msgs;
    packet.rx_errors = rx_errors;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WIRELESS_STATISTICS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WIRELESS_STATISTICS_MIN_LEN, MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN, MAVLINK_MSG_ID_WIRELESS_STATISTICS_CRC);
}

/**
 * @brief Encode a wireless_statistics struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wireless_statistics C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wireless_statistics_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wireless_statistics_t* wireless_statistics)
{
    return mavlink_msg_wireless_statistics_pack(system_id, component_id, msg, wireless_statistics->rx_bytes, wireless_statistics->tx_bytes, wireless_statistics->rx_msgs, wireless_statistics->rx_errors);
}

/**
 * @brief Encode a wireless_statistics struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wireless_statistics C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wireless_statistics_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_wireless_statistics_t* wireless_statistics)
{
    return mavlink_msg_wireless_statistics_pack_chan(system_id, component_id, chan, msg, wireless_statistics->rx_bytes, wireless_statistics->tx_bytes, wireless_statistics->rx_msgs, wireless_statistics->rx_errors);
}

/**
 * @brief Send a wireless_statistics message
 * @param chan MAVLink channel to send the message
 *
 * @param rx_bytes Nr of received bytes
 * @param tx_bytes Nr of sent bytes
 * @param rx_msgs Nr of received wireless messages of pwrmaster
 * @param rx_errors Nr of transmitted wireless messages of pwrmaster
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wireless_statistics_send(mavlink_channel_t chan, uint32_t rx_bytes, uint32_t tx_bytes, uint16_t rx_msgs, uint16_t rx_errors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN];
    _mav_put_uint32_t(buf, 0, rx_bytes);
    _mav_put_uint32_t(buf, 4, tx_bytes);
    _mav_put_uint16_t(buf, 8, rx_msgs);
    _mav_put_uint16_t(buf, 10, rx_errors);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIRELESS_STATISTICS, buf, MAVLINK_MSG_ID_WIRELESS_STATISTICS_MIN_LEN, MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN, MAVLINK_MSG_ID_WIRELESS_STATISTICS_CRC);
#else
    mavlink_wireless_statistics_t packet;
    packet.rx_bytes = rx_bytes;
    packet.tx_bytes = tx_bytes;
    packet.rx_msgs = rx_msgs;
    packet.rx_errors = rx_errors;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIRELESS_STATISTICS, (const char *)&packet, MAVLINK_MSG_ID_WIRELESS_STATISTICS_MIN_LEN, MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN, MAVLINK_MSG_ID_WIRELESS_STATISTICS_CRC);
#endif
}

/**
 * @brief Send a wireless_statistics message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_wireless_statistics_send_struct(mavlink_channel_t chan, const mavlink_wireless_statistics_t* wireless_statistics)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_wireless_statistics_send(chan, wireless_statistics->rx_bytes, wireless_statistics->tx_bytes, wireless_statistics->rx_msgs, wireless_statistics->rx_errors);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIRELESS_STATISTICS, (const char *)wireless_statistics, MAVLINK_MSG_ID_WIRELESS_STATISTICS_MIN_LEN, MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN, MAVLINK_MSG_ID_WIRELESS_STATISTICS_CRC);
#endif
}

#if MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_wireless_statistics_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t rx_bytes, uint32_t tx_bytes, uint16_t rx_msgs, uint16_t rx_errors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, rx_bytes);
    _mav_put_uint32_t(buf, 4, tx_bytes);
    _mav_put_uint16_t(buf, 8, rx_msgs);
    _mav_put_uint16_t(buf, 10, rx_errors);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIRELESS_STATISTICS, buf, MAVLINK_MSG_ID_WIRELESS_STATISTICS_MIN_LEN, MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN, MAVLINK_MSG_ID_WIRELESS_STATISTICS_CRC);
#else
    mavlink_wireless_statistics_t *packet = (mavlink_wireless_statistics_t *)msgbuf;
    packet->rx_bytes = rx_bytes;
    packet->tx_bytes = tx_bytes;
    packet->rx_msgs = rx_msgs;
    packet->rx_errors = rx_errors;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIRELESS_STATISTICS, (const char *)packet, MAVLINK_MSG_ID_WIRELESS_STATISTICS_MIN_LEN, MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN, MAVLINK_MSG_ID_WIRELESS_STATISTICS_CRC);
#endif
}
#endif

#endif

// MESSAGE WIRELESS_STATISTICS UNPACKING


/**
 * @brief Get field rx_bytes from wireless_statistics message
 *
 * @return Nr of received bytes
 */
static inline uint32_t mavlink_msg_wireless_statistics_get_rx_bytes(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field tx_bytes from wireless_statistics message
 *
 * @return Nr of sent bytes
 */
static inline uint32_t mavlink_msg_wireless_statistics_get_tx_bytes(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field rx_msgs from wireless_statistics message
 *
 * @return Nr of received wireless messages of pwrmaster
 */
static inline uint16_t mavlink_msg_wireless_statistics_get_rx_msgs(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field rx_errors from wireless_statistics message
 *
 * @return Nr of transmitted wireless messages of pwrmaster
 */
static inline uint16_t mavlink_msg_wireless_statistics_get_rx_errors(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Decode a wireless_statistics message into a struct
 *
 * @param msg The message to decode
 * @param wireless_statistics C-struct to decode the message contents into
 */
static inline void mavlink_msg_wireless_statistics_decode(const mavlink_message_t* msg, mavlink_wireless_statistics_t* wireless_statistics)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    wireless_statistics->rx_bytes = mavlink_msg_wireless_statistics_get_rx_bytes(msg);
    wireless_statistics->tx_bytes = mavlink_msg_wireless_statistics_get_tx_bytes(msg);
    wireless_statistics->rx_msgs = mavlink_msg_wireless_statistics_get_rx_msgs(msg);
    wireless_statistics->rx_errors = mavlink_msg_wireless_statistics_get_rx_errors(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN? msg->len : MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN;
        memset(wireless_statistics, 0, MAVLINK_MSG_ID_WIRELESS_STATISTICS_LEN);
    memcpy(wireless_statistics, _MAV_PAYLOAD(msg), len);
#endif
}
