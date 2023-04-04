#pragma once
// MESSAGE PX4GPIO_SET_CMD PACKING

#define MAVLINK_MSG_ID_PX4GPIO_SET_CMD 21

MAVPACKED(
typedef struct __mavlink_px4gpio_set_cmd_t {
 int8_t main_enable; /*<  command for main_enable pin*/
 int8_t enable_can_pwr; /*<  command for main_enable pin*/
 int8_t enable_exdev; /*<  command for main_enable pin*/
 int8_t enable_5v; /*<  command for main_enable pin*/
 int8_t enable_12v ; /*<  command for main_enable pin*/
 int8_t enable_15v; /*<  command for main_enable pin*/
 int8_t mainled_enable; /*<  command for main_enable pin*/
}) mavlink_px4gpio_set_cmd_t;

#define MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN 7
#define MAVLINK_MSG_ID_PX4GPIO_SET_CMD_MIN_LEN 7
#define MAVLINK_MSG_ID_21_LEN 7
#define MAVLINK_MSG_ID_21_MIN_LEN 7

#define MAVLINK_MSG_ID_PX4GPIO_SET_CMD_CRC 63
#define MAVLINK_MSG_ID_21_CRC 63



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PX4GPIO_SET_CMD { \
    21, \
    "PX4GPIO_SET_CMD", \
    7, \
    {  { "main_enable", NULL, MAVLINK_TYPE_INT8_T, 0, 0, offsetof(mavlink_px4gpio_set_cmd_t, main_enable) }, \
         { "enable_can_pwr", NULL, MAVLINK_TYPE_INT8_T, 0, 1, offsetof(mavlink_px4gpio_set_cmd_t, enable_can_pwr) }, \
         { "enable_exdev", NULL, MAVLINK_TYPE_INT8_T, 0, 2, offsetof(mavlink_px4gpio_set_cmd_t, enable_exdev) }, \
         { "enable_5v", NULL, MAVLINK_TYPE_INT8_T, 0, 3, offsetof(mavlink_px4gpio_set_cmd_t, enable_5v) }, \
         { "enable_12v ", NULL, MAVLINK_TYPE_INT8_T, 0, 4, offsetof(mavlink_px4gpio_set_cmd_t, enable_12v ) }, \
         { "enable_15v", NULL, MAVLINK_TYPE_INT8_T, 0, 5, offsetof(mavlink_px4gpio_set_cmd_t, enable_15v) }, \
         { "mainled_enable", NULL, MAVLINK_TYPE_INT8_T, 0, 6, offsetof(mavlink_px4gpio_set_cmd_t, mainled_enable) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PX4GPIO_SET_CMD { \
    "PX4GPIO_SET_CMD", \
    7, \
    {  { "main_enable", NULL, MAVLINK_TYPE_INT8_T, 0, 0, offsetof(mavlink_px4gpio_set_cmd_t, main_enable) }, \
         { "enable_can_pwr", NULL, MAVLINK_TYPE_INT8_T, 0, 1, offsetof(mavlink_px4gpio_set_cmd_t, enable_can_pwr) }, \
         { "enable_exdev", NULL, MAVLINK_TYPE_INT8_T, 0, 2, offsetof(mavlink_px4gpio_set_cmd_t, enable_exdev) }, \
         { "enable_5v", NULL, MAVLINK_TYPE_INT8_T, 0, 3, offsetof(mavlink_px4gpio_set_cmd_t, enable_5v) }, \
         { "enable_12v ", NULL, MAVLINK_TYPE_INT8_T, 0, 4, offsetof(mavlink_px4gpio_set_cmd_t, enable_12v ) }, \
         { "enable_15v", NULL, MAVLINK_TYPE_INT8_T, 0, 5, offsetof(mavlink_px4gpio_set_cmd_t, enable_15v) }, \
         { "mainled_enable", NULL, MAVLINK_TYPE_INT8_T, 0, 6, offsetof(mavlink_px4gpio_set_cmd_t, mainled_enable) }, \
         } \
}
#endif

/**
 * @brief Pack a px4gpio_set_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param main_enable  command for main_enable pin
 * @param enable_can_pwr  command for main_enable pin
 * @param enable_exdev  command for main_enable pin
 * @param enable_5v  command for main_enable pin
 * @param enable_12v   command for main_enable pin
 * @param enable_15v  command for main_enable pin
 * @param mainled_enable  command for main_enable pin
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4gpio_set_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int8_t main_enable, int8_t enable_can_pwr, int8_t enable_exdev, int8_t enable_5v, int8_t enable_12v , int8_t enable_15v, int8_t mainled_enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN];
    _mav_put_int8_t(buf, 0, main_enable);
    _mav_put_int8_t(buf, 1, enable_can_pwr);
    _mav_put_int8_t(buf, 2, enable_exdev);
    _mav_put_int8_t(buf, 3, enable_5v);
    _mav_put_int8_t(buf, 4, enable_12v );
    _mav_put_int8_t(buf, 5, enable_15v);
    _mav_put_int8_t(buf, 6, mainled_enable);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN);
#else
    mavlink_px4gpio_set_cmd_t packet;
    packet.main_enable = main_enable;
    packet.enable_can_pwr = enable_can_pwr;
    packet.enable_exdev = enable_exdev;
    packet.enable_5v = enable_5v;
    packet.enable_12v  = enable_12v ;
    packet.enable_15v = enable_15v;
    packet.mainled_enable = mainled_enable;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4GPIO_SET_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_MIN_LEN, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_CRC);
}

/**
 * @brief Pack a px4gpio_set_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param main_enable  command for main_enable pin
 * @param enable_can_pwr  command for main_enable pin
 * @param enable_exdev  command for main_enable pin
 * @param enable_5v  command for main_enable pin
 * @param enable_12v   command for main_enable pin
 * @param enable_15v  command for main_enable pin
 * @param mainled_enable  command for main_enable pin
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4gpio_set_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int8_t main_enable,int8_t enable_can_pwr,int8_t enable_exdev,int8_t enable_5v,int8_t enable_12v ,int8_t enable_15v,int8_t mainled_enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN];
    _mav_put_int8_t(buf, 0, main_enable);
    _mav_put_int8_t(buf, 1, enable_can_pwr);
    _mav_put_int8_t(buf, 2, enable_exdev);
    _mav_put_int8_t(buf, 3, enable_5v);
    _mav_put_int8_t(buf, 4, enable_12v );
    _mav_put_int8_t(buf, 5, enable_15v);
    _mav_put_int8_t(buf, 6, mainled_enable);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN);
#else
    mavlink_px4gpio_set_cmd_t packet;
    packet.main_enable = main_enable;
    packet.enable_can_pwr = enable_can_pwr;
    packet.enable_exdev = enable_exdev;
    packet.enable_5v = enable_5v;
    packet.enable_12v  = enable_12v ;
    packet.enable_15v = enable_15v;
    packet.mainled_enable = mainled_enable;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4GPIO_SET_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_MIN_LEN, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_CRC);
}

/**
 * @brief Encode a px4gpio_set_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param px4gpio_set_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4gpio_set_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_px4gpio_set_cmd_t* px4gpio_set_cmd)
{
    return mavlink_msg_px4gpio_set_cmd_pack(system_id, component_id, msg, px4gpio_set_cmd->main_enable, px4gpio_set_cmd->enable_can_pwr, px4gpio_set_cmd->enable_exdev, px4gpio_set_cmd->enable_5v, px4gpio_set_cmd->enable_12v , px4gpio_set_cmd->enable_15v, px4gpio_set_cmd->mainled_enable);
}

/**
 * @brief Encode a px4gpio_set_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param px4gpio_set_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4gpio_set_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_px4gpio_set_cmd_t* px4gpio_set_cmd)
{
    return mavlink_msg_px4gpio_set_cmd_pack_chan(system_id, component_id, chan, msg, px4gpio_set_cmd->main_enable, px4gpio_set_cmd->enable_can_pwr, px4gpio_set_cmd->enable_exdev, px4gpio_set_cmd->enable_5v, px4gpio_set_cmd->enable_12v , px4gpio_set_cmd->enable_15v, px4gpio_set_cmd->mainled_enable);
}

/**
 * @brief Send a px4gpio_set_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param main_enable  command for main_enable pin
 * @param enable_can_pwr  command for main_enable pin
 * @param enable_exdev  command for main_enable pin
 * @param enable_5v  command for main_enable pin
 * @param enable_12v   command for main_enable pin
 * @param enable_15v  command for main_enable pin
 * @param mainled_enable  command for main_enable pin
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_px4gpio_set_cmd_send(mavlink_channel_t chan, int8_t main_enable, int8_t enable_can_pwr, int8_t enable_exdev, int8_t enable_5v, int8_t enable_12v , int8_t enable_15v, int8_t mainled_enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN];
    _mav_put_int8_t(buf, 0, main_enable);
    _mav_put_int8_t(buf, 1, enable_can_pwr);
    _mav_put_int8_t(buf, 2, enable_exdev);
    _mav_put_int8_t(buf, 3, enable_5v);
    _mav_put_int8_t(buf, 4, enable_12v );
    _mav_put_int8_t(buf, 5, enable_15v);
    _mav_put_int8_t(buf, 6, mainled_enable);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4GPIO_SET_CMD, buf, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_MIN_LEN, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_CRC);
#else
    mavlink_px4gpio_set_cmd_t packet;
    packet.main_enable = main_enable;
    packet.enable_can_pwr = enable_can_pwr;
    packet.enable_exdev = enable_exdev;
    packet.enable_5v = enable_5v;
    packet.enable_12v  = enable_12v ;
    packet.enable_15v = enable_15v;
    packet.mainled_enable = mainled_enable;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4GPIO_SET_CMD, (const char *)&packet, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_MIN_LEN, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_CRC);
#endif
}

/**
 * @brief Send a px4gpio_set_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_px4gpio_set_cmd_send_struct(mavlink_channel_t chan, const mavlink_px4gpio_set_cmd_t* px4gpio_set_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_px4gpio_set_cmd_send(chan, px4gpio_set_cmd->main_enable, px4gpio_set_cmd->enable_can_pwr, px4gpio_set_cmd->enable_exdev, px4gpio_set_cmd->enable_5v, px4gpio_set_cmd->enable_12v , px4gpio_set_cmd->enable_15v, px4gpio_set_cmd->mainled_enable);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4GPIO_SET_CMD, (const char *)px4gpio_set_cmd, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_MIN_LEN, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_px4gpio_set_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int8_t main_enable, int8_t enable_can_pwr, int8_t enable_exdev, int8_t enable_5v, int8_t enable_12v , int8_t enable_15v, int8_t mainled_enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int8_t(buf, 0, main_enable);
    _mav_put_int8_t(buf, 1, enable_can_pwr);
    _mav_put_int8_t(buf, 2, enable_exdev);
    _mav_put_int8_t(buf, 3, enable_5v);
    _mav_put_int8_t(buf, 4, enable_12v );
    _mav_put_int8_t(buf, 5, enable_15v);
    _mav_put_int8_t(buf, 6, mainled_enable);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4GPIO_SET_CMD, buf, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_MIN_LEN, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_CRC);
#else
    mavlink_px4gpio_set_cmd_t *packet = (mavlink_px4gpio_set_cmd_t *)msgbuf;
    packet->main_enable = main_enable;
    packet->enable_can_pwr = enable_can_pwr;
    packet->enable_exdev = enable_exdev;
    packet->enable_5v = enable_5v;
    packet->enable_12v  = enable_12v ;
    packet->enable_15v = enable_15v;
    packet->mainled_enable = mainled_enable;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4GPIO_SET_CMD, (const char *)packet, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_MIN_LEN, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE PX4GPIO_SET_CMD UNPACKING


/**
 * @brief Get field main_enable from px4gpio_set_cmd message
 *
 * @return  command for main_enable pin
 */
static inline int8_t mavlink_msg_px4gpio_set_cmd_get_main_enable(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  0);
}

/**
 * @brief Get field enable_can_pwr from px4gpio_set_cmd message
 *
 * @return  command for main_enable pin
 */
static inline int8_t mavlink_msg_px4gpio_set_cmd_get_enable_can_pwr(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  1);
}

/**
 * @brief Get field enable_exdev from px4gpio_set_cmd message
 *
 * @return  command for main_enable pin
 */
static inline int8_t mavlink_msg_px4gpio_set_cmd_get_enable_exdev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  2);
}

/**
 * @brief Get field enable_5v from px4gpio_set_cmd message
 *
 * @return  command for main_enable pin
 */
static inline int8_t mavlink_msg_px4gpio_set_cmd_get_enable_5v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  3);
}

/**
 * @brief Get field enable_12v  from px4gpio_set_cmd message
 *
 * @return  command for main_enable pin
 */
static inline int8_t mavlink_msg_px4gpio_set_cmd_get_enable_12v (const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  4);
}

/**
 * @brief Get field enable_15v from px4gpio_set_cmd message
 *
 * @return  command for main_enable pin
 */
static inline int8_t mavlink_msg_px4gpio_set_cmd_get_enable_15v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  5);
}

/**
 * @brief Get field mainled_enable from px4gpio_set_cmd message
 *
 * @return  command for main_enable pin
 */
static inline int8_t mavlink_msg_px4gpio_set_cmd_get_mainled_enable(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  6);
}

/**
 * @brief Decode a px4gpio_set_cmd message into a struct
 *
 * @param msg The message to decode
 * @param px4gpio_set_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_px4gpio_set_cmd_decode(const mavlink_message_t* msg, mavlink_px4gpio_set_cmd_t* px4gpio_set_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    px4gpio_set_cmd->main_enable = mavlink_msg_px4gpio_set_cmd_get_main_enable(msg);
    px4gpio_set_cmd->enable_can_pwr = mavlink_msg_px4gpio_set_cmd_get_enable_can_pwr(msg);
    px4gpio_set_cmd->enable_exdev = mavlink_msg_px4gpio_set_cmd_get_enable_exdev(msg);
    px4gpio_set_cmd->enable_5v = mavlink_msg_px4gpio_set_cmd_get_enable_5v(msg);
    px4gpio_set_cmd->enable_12v  = mavlink_msg_px4gpio_set_cmd_get_enable_12v (msg);
    px4gpio_set_cmd->enable_15v = mavlink_msg_px4gpio_set_cmd_get_enable_15v(msg);
    px4gpio_set_cmd->mainled_enable = mavlink_msg_px4gpio_set_cmd_get_mainled_enable(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN? msg->len : MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN;
        memset(px4gpio_set_cmd, 0, MAVLINK_MSG_ID_PX4GPIO_SET_CMD_LEN);
    memcpy(px4gpio_set_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
