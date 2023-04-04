#pragma once
// MESSAGE SLAVE_ENABLE_CMD PACKING

#define MAVLINK_MSG_ID_SLAVE_ENABLE_CMD 22

MAVPACKED(
typedef struct __mavlink_slave_enable_cmd_t {
 uint8_t slave_id; /*< slave ID of slave*/
 uint8_t set_cmd; /*< current pin set/status choose from pin_cmd enum*/
}) mavlink_slave_enable_cmd_t;

#define MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN 2
#define MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_MIN_LEN 2
#define MAVLINK_MSG_ID_22_LEN 2
#define MAVLINK_MSG_ID_22_MIN_LEN 2

#define MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_CRC 201
#define MAVLINK_MSG_ID_22_CRC 201



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SLAVE_ENABLE_CMD { \
    22, \
    "SLAVE_ENABLE_CMD", \
    2, \
    {  { "slave_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_slave_enable_cmd_t, slave_id) }, \
         { "set_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_slave_enable_cmd_t, set_cmd) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SLAVE_ENABLE_CMD { \
    "SLAVE_ENABLE_CMD", \
    2, \
    {  { "slave_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_slave_enable_cmd_t, slave_id) }, \
         { "set_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_slave_enable_cmd_t, set_cmd) }, \
         } \
}
#endif

/**
 * @brief Pack a slave_enable_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param slave_id slave ID of slave
 * @param set_cmd current pin set/status choose from pin_cmd enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slave_enable_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t slave_id, uint8_t set_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN];
    _mav_put_uint8_t(buf, 0, slave_id);
    _mav_put_uint8_t(buf, 1, set_cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN);
#else
    mavlink_slave_enable_cmd_t packet;
    packet.slave_id = slave_id;
    packet.set_cmd = set_cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SLAVE_ENABLE_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_MIN_LEN, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_CRC);
}

/**
 * @brief Pack a slave_enable_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param slave_id slave ID of slave
 * @param set_cmd current pin set/status choose from pin_cmd enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slave_enable_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t slave_id,uint8_t set_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN];
    _mav_put_uint8_t(buf, 0, slave_id);
    _mav_put_uint8_t(buf, 1, set_cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN);
#else
    mavlink_slave_enable_cmd_t packet;
    packet.slave_id = slave_id;
    packet.set_cmd = set_cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SLAVE_ENABLE_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_MIN_LEN, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_CRC);
}

/**
 * @brief Encode a slave_enable_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param slave_enable_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_slave_enable_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_slave_enable_cmd_t* slave_enable_cmd)
{
    return mavlink_msg_slave_enable_cmd_pack(system_id, component_id, msg, slave_enable_cmd->slave_id, slave_enable_cmd->set_cmd);
}

/**
 * @brief Encode a slave_enable_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param slave_enable_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_slave_enable_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_slave_enable_cmd_t* slave_enable_cmd)
{
    return mavlink_msg_slave_enable_cmd_pack_chan(system_id, component_id, chan, msg, slave_enable_cmd->slave_id, slave_enable_cmd->set_cmd);
}

/**
 * @brief Send a slave_enable_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param slave_id slave ID of slave
 * @param set_cmd current pin set/status choose from pin_cmd enum
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_slave_enable_cmd_send(mavlink_channel_t chan, uint8_t slave_id, uint8_t set_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN];
    _mav_put_uint8_t(buf, 0, slave_id);
    _mav_put_uint8_t(buf, 1, set_cmd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD, buf, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_MIN_LEN, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_CRC);
#else
    mavlink_slave_enable_cmd_t packet;
    packet.slave_id = slave_id;
    packet.set_cmd = set_cmd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD, (const char *)&packet, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_MIN_LEN, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_CRC);
#endif
}

/**
 * @brief Send a slave_enable_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_slave_enable_cmd_send_struct(mavlink_channel_t chan, const mavlink_slave_enable_cmd_t* slave_enable_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_slave_enable_cmd_send(chan, slave_enable_cmd->slave_id, slave_enable_cmd->set_cmd);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD, (const char *)slave_enable_cmd, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_MIN_LEN, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_slave_enable_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t slave_id, uint8_t set_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, slave_id);
    _mav_put_uint8_t(buf, 1, set_cmd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD, buf, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_MIN_LEN, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_CRC);
#else
    mavlink_slave_enable_cmd_t *packet = (mavlink_slave_enable_cmd_t *)msgbuf;
    packet->slave_id = slave_id;
    packet->set_cmd = set_cmd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD, (const char *)packet, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_MIN_LEN, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE SLAVE_ENABLE_CMD UNPACKING


/**
 * @brief Get field slave_id from slave_enable_cmd message
 *
 * @return slave ID of slave
 */
static inline uint8_t mavlink_msg_slave_enable_cmd_get_slave_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field set_cmd from slave_enable_cmd message
 *
 * @return current pin set/status choose from pin_cmd enum
 */
static inline uint8_t mavlink_msg_slave_enable_cmd_get_set_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a slave_enable_cmd message into a struct
 *
 * @param msg The message to decode
 * @param slave_enable_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_slave_enable_cmd_decode(const mavlink_message_t* msg, mavlink_slave_enable_cmd_t* slave_enable_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    slave_enable_cmd->slave_id = mavlink_msg_slave_enable_cmd_get_slave_id(msg);
    slave_enable_cmd->set_cmd = mavlink_msg_slave_enable_cmd_get_set_cmd(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN? msg->len : MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN;
        memset(slave_enable_cmd, 0, MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_LEN);
    memcpy(slave_enable_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
