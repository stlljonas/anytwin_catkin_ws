#pragma once
// MESSAGE MASTER_MAVLINK_NOTIFICATION PACKING

#define MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION 8

MAVPACKED(
typedef struct __mavlink_master_mavlink_notification_t {
 uint32_t code; /*< Error Code*/
 uint8_t level; /*< Notification Level, choose from NOTIFICATION_LEVEL*/
 char name[10]; /*< Short description of the Notification*/
 char description[50]; /*< More Detailon Notification*/
}) mavlink_master_mavlink_notification_t;

#define MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN 65
#define MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_MIN_LEN 65
#define MAVLINK_MSG_ID_8_LEN 65
#define MAVLINK_MSG_ID_8_MIN_LEN 65

#define MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_CRC 50
#define MAVLINK_MSG_ID_8_CRC 50

#define MAVLINK_MSG_MASTER_MAVLINK_NOTIFICATION_FIELD_NAME_LEN 10
#define MAVLINK_MSG_MASTER_MAVLINK_NOTIFICATION_FIELD_DESCRIPTION_LEN 50

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MASTER_MAVLINK_NOTIFICATION { \
    8, \
    "MASTER_MAVLINK_NOTIFICATION", \
    4, \
    {  { "code", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_master_mavlink_notification_t, code) }, \
         { "level", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_master_mavlink_notification_t, level) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 10, 5, offsetof(mavlink_master_mavlink_notification_t, name) }, \
         { "description", NULL, MAVLINK_TYPE_CHAR, 50, 15, offsetof(mavlink_master_mavlink_notification_t, description) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MASTER_MAVLINK_NOTIFICATION { \
    "MASTER_MAVLINK_NOTIFICATION", \
    4, \
    {  { "code", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_master_mavlink_notification_t, code) }, \
         { "level", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_master_mavlink_notification_t, level) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 10, 5, offsetof(mavlink_master_mavlink_notification_t, name) }, \
         { "description", NULL, MAVLINK_TYPE_CHAR, 50, 15, offsetof(mavlink_master_mavlink_notification_t, description) }, \
         } \
}
#endif

/**
 * @brief Pack a master_mavlink_notification message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param code Error Code
 * @param level Notification Level, choose from NOTIFICATION_LEVEL
 * @param name Short description of the Notification
 * @param description More Detailon Notification
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_master_mavlink_notification_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t code, uint8_t level, const char *name, const char *description)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN];
    _mav_put_uint32_t(buf, 0, code);
    _mav_put_uint8_t(buf, 4, level);
    _mav_put_char_array(buf, 5, name, 10);
    _mav_put_char_array(buf, 15, description, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN);
#else
    mavlink_master_mavlink_notification_t packet;
    packet.code = code;
    packet.level = level;
    mav_array_memcpy(packet.name, name, sizeof(char)*10);
    mav_array_memcpy(packet.description, description, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_MIN_LEN, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_CRC);
}

/**
 * @brief Pack a master_mavlink_notification message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param code Error Code
 * @param level Notification Level, choose from NOTIFICATION_LEVEL
 * @param name Short description of the Notification
 * @param description More Detailon Notification
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_master_mavlink_notification_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t code,uint8_t level,const char *name,const char *description)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN];
    _mav_put_uint32_t(buf, 0, code);
    _mav_put_uint8_t(buf, 4, level);
    _mav_put_char_array(buf, 5, name, 10);
    _mav_put_char_array(buf, 15, description, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN);
#else
    mavlink_master_mavlink_notification_t packet;
    packet.code = code;
    packet.level = level;
    mav_array_memcpy(packet.name, name, sizeof(char)*10);
    mav_array_memcpy(packet.description, description, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_MIN_LEN, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_CRC);
}

/**
 * @brief Encode a master_mavlink_notification struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param master_mavlink_notification C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_master_mavlink_notification_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_master_mavlink_notification_t* master_mavlink_notification)
{
    return mavlink_msg_master_mavlink_notification_pack(system_id, component_id, msg, master_mavlink_notification->code, master_mavlink_notification->level, master_mavlink_notification->name, master_mavlink_notification->description);
}

/**
 * @brief Encode a master_mavlink_notification struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param master_mavlink_notification C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_master_mavlink_notification_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_master_mavlink_notification_t* master_mavlink_notification)
{
    return mavlink_msg_master_mavlink_notification_pack_chan(system_id, component_id, chan, msg, master_mavlink_notification->code, master_mavlink_notification->level, master_mavlink_notification->name, master_mavlink_notification->description);
}

/**
 * @brief Send a master_mavlink_notification message
 * @param chan MAVLink channel to send the message
 *
 * @param code Error Code
 * @param level Notification Level, choose from NOTIFICATION_LEVEL
 * @param name Short description of the Notification
 * @param description More Detailon Notification
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_master_mavlink_notification_send(mavlink_channel_t chan, uint32_t code, uint8_t level, const char *name, const char *description)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN];
    _mav_put_uint32_t(buf, 0, code);
    _mav_put_uint8_t(buf, 4, level);
    _mav_put_char_array(buf, 5, name, 10);
    _mav_put_char_array(buf, 15, description, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION, buf, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_MIN_LEN, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_CRC);
#else
    mavlink_master_mavlink_notification_t packet;
    packet.code = code;
    packet.level = level;
    mav_array_memcpy(packet.name, name, sizeof(char)*10);
    mav_array_memcpy(packet.description, description, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION, (const char *)&packet, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_MIN_LEN, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_CRC);
#endif
}

/**
 * @brief Send a master_mavlink_notification message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_master_mavlink_notification_send_struct(mavlink_channel_t chan, const mavlink_master_mavlink_notification_t* master_mavlink_notification)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_master_mavlink_notification_send(chan, master_mavlink_notification->code, master_mavlink_notification->level, master_mavlink_notification->name, master_mavlink_notification->description);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION, (const char *)master_mavlink_notification, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_MIN_LEN, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_master_mavlink_notification_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t code, uint8_t level, const char *name, const char *description)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, code);
    _mav_put_uint8_t(buf, 4, level);
    _mav_put_char_array(buf, 5, name, 10);
    _mav_put_char_array(buf, 15, description, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION, buf, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_MIN_LEN, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_CRC);
#else
    mavlink_master_mavlink_notification_t *packet = (mavlink_master_mavlink_notification_t *)msgbuf;
    packet->code = code;
    packet->level = level;
    mav_array_memcpy(packet->name, name, sizeof(char)*10);
    mav_array_memcpy(packet->description, description, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION, (const char *)packet, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_MIN_LEN, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_CRC);
#endif
}
#endif

#endif

// MESSAGE MASTER_MAVLINK_NOTIFICATION UNPACKING


/**
 * @brief Get field code from master_mavlink_notification message
 *
 * @return Error Code
 */
static inline uint32_t mavlink_msg_master_mavlink_notification_get_code(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field level from master_mavlink_notification message
 *
 * @return Notification Level, choose from NOTIFICATION_LEVEL
 */
static inline uint8_t mavlink_msg_master_mavlink_notification_get_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field name from master_mavlink_notification message
 *
 * @return Short description of the Notification
 */
static inline uint16_t mavlink_msg_master_mavlink_notification_get_name(const mavlink_message_t* msg, char *name)
{
    return _MAV_RETURN_char_array(msg, name, 10,  5);
}

/**
 * @brief Get field description from master_mavlink_notification message
 *
 * @return More Detailon Notification
 */
static inline uint16_t mavlink_msg_master_mavlink_notification_get_description(const mavlink_message_t* msg, char *description)
{
    return _MAV_RETURN_char_array(msg, description, 50,  15);
}

/**
 * @brief Decode a master_mavlink_notification message into a struct
 *
 * @param msg The message to decode
 * @param master_mavlink_notification C-struct to decode the message contents into
 */
static inline void mavlink_msg_master_mavlink_notification_decode(const mavlink_message_t* msg, mavlink_master_mavlink_notification_t* master_mavlink_notification)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    master_mavlink_notification->code = mavlink_msg_master_mavlink_notification_get_code(msg);
    master_mavlink_notification->level = mavlink_msg_master_mavlink_notification_get_level(msg);
    mavlink_msg_master_mavlink_notification_get_name(msg, master_mavlink_notification->name);
    mavlink_msg_master_mavlink_notification_get_description(msg, master_mavlink_notification->description);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN? msg->len : MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN;
        memset(master_mavlink_notification, 0, MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_LEN);
    memcpy(master_mavlink_notification, _MAV_PAYLOAD(msg), len);
#endif
}
