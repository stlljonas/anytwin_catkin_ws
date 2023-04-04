#pragma once
// MESSAGE HEARTBEAT PACKING

#define MAVLINK_MSG_ID_HEARTBEAT 0

MAVPACKED(
typedef struct __mavlink_heartbeat_t {
 uint16_t subsystem_status; /*< Subsystem's status that comes from master*/
 uint16_t errors; /*< Master module's specific errors*/
 uint16_t bms_voltage_mv; /*< Battery Voltage in mV*/
 int16_t bms_current_ma; /*< Battery Current in mA*/
 uint8_t type; /*< Type of the heartbeat sender. defined in ANYMAL_SENDER_TYPES ENUM*/
 uint8_t master_status; /*< System status flag, comes from master*/
 uint8_t bms_working; /*< Specifies if bms is working*/
 uint8_t bms_stateofcharge; /*< Battery State of Charge %*/
 uint8_t mavlink_version; /*< MAVLink version*/
}) mavlink_heartbeat_t;

#define MAVLINK_MSG_ID_HEARTBEAT_LEN 13
#define MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN 13
#define MAVLINK_MSG_ID_0_LEN 13
#define MAVLINK_MSG_ID_0_MIN_LEN 13

#define MAVLINK_MSG_ID_HEARTBEAT_CRC 202
#define MAVLINK_MSG_ID_0_CRC 202



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HEARTBEAT { \
    0, \
    "HEARTBEAT", \
    9, \
    {  { "subsystem_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_heartbeat_t, subsystem_status) }, \
         { "errors", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_heartbeat_t, errors) }, \
         { "bms_voltage_mv", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_heartbeat_t, bms_voltage_mv) }, \
         { "bms_current_ma", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_heartbeat_t, bms_current_ma) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_heartbeat_t, type) }, \
         { "master_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_heartbeat_t, master_status) }, \
         { "bms_working", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_heartbeat_t, bms_working) }, \
         { "bms_stateofcharge", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_heartbeat_t, bms_stateofcharge) }, \
         { "mavlink_version", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_heartbeat_t, mavlink_version) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HEARTBEAT { \
    "HEARTBEAT", \
    9, \
    {  { "subsystem_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_heartbeat_t, subsystem_status) }, \
         { "errors", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_heartbeat_t, errors) }, \
         { "bms_voltage_mv", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_heartbeat_t, bms_voltage_mv) }, \
         { "bms_current_ma", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_heartbeat_t, bms_current_ma) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_heartbeat_t, type) }, \
         { "master_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_heartbeat_t, master_status) }, \
         { "bms_working", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_heartbeat_t, bms_working) }, \
         { "bms_stateofcharge", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_heartbeat_t, bms_stateofcharge) }, \
         { "mavlink_version", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_heartbeat_t, mavlink_version) }, \
         } \
}
#endif

/**
 * @brief Pack a heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type Type of the heartbeat sender. defined in ANYMAL_SENDER_TYPES ENUM
 * @param master_status System status flag, comes from master
 * @param subsystem_status Subsystem's status that comes from master
 * @param errors Master module's specific errors
 * @param bms_working Specifies if bms is working
 * @param bms_voltage_mv Battery Voltage in mV
 * @param bms_current_ma Battery Current in mA
 * @param bms_stateofcharge Battery State of Charge %
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t type, uint8_t master_status, uint16_t subsystem_status, uint16_t errors, uint8_t bms_working, uint16_t bms_voltage_mv, int16_t bms_current_ma, uint8_t bms_stateofcharge)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
    _mav_put_uint16_t(buf, 0, subsystem_status);
    _mav_put_uint16_t(buf, 2, errors);
    _mav_put_uint16_t(buf, 4, bms_voltage_mv);
    _mav_put_int16_t(buf, 6, bms_current_ma);
    _mav_put_uint8_t(buf, 8, type);
    _mav_put_uint8_t(buf, 9, master_status);
    _mav_put_uint8_t(buf, 10, bms_working);
    _mav_put_uint8_t(buf, 11, bms_stateofcharge);
    _mav_put_uint8_t(buf, 12, 2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#else
    mavlink_heartbeat_t packet;
    packet.subsystem_status = subsystem_status;
    packet.errors = errors;
    packet.bms_voltage_mv = bms_voltage_mv;
    packet.bms_current_ma = bms_current_ma;
    packet.type = type;
    packet.master_status = master_status;
    packet.bms_working = bms_working;
    packet.bms_stateofcharge = bms_stateofcharge;
    packet.mavlink_version = 2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
}

/**
 * @brief Pack a heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param type Type of the heartbeat sender. defined in ANYMAL_SENDER_TYPES ENUM
 * @param master_status System status flag, comes from master
 * @param subsystem_status Subsystem's status that comes from master
 * @param errors Master module's specific errors
 * @param bms_working Specifies if bms is working
 * @param bms_voltage_mv Battery Voltage in mV
 * @param bms_current_ma Battery Current in mA
 * @param bms_stateofcharge Battery State of Charge %
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t type,uint8_t master_status,uint16_t subsystem_status,uint16_t errors,uint8_t bms_working,uint16_t bms_voltage_mv,int16_t bms_current_ma,uint8_t bms_stateofcharge)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
    _mav_put_uint16_t(buf, 0, subsystem_status);
    _mav_put_uint16_t(buf, 2, errors);
    _mav_put_uint16_t(buf, 4, bms_voltage_mv);
    _mav_put_int16_t(buf, 6, bms_current_ma);
    _mav_put_uint8_t(buf, 8, type);
    _mav_put_uint8_t(buf, 9, master_status);
    _mav_put_uint8_t(buf, 10, bms_working);
    _mav_put_uint8_t(buf, 11, bms_stateofcharge);
    _mav_put_uint8_t(buf, 12, 2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#else
    mavlink_heartbeat_t packet;
    packet.subsystem_status = subsystem_status;
    packet.errors = errors;
    packet.bms_voltage_mv = bms_voltage_mv;
    packet.bms_current_ma = bms_current_ma;
    packet.type = type;
    packet.master_status = master_status;
    packet.bms_working = bms_working;
    packet.bms_stateofcharge = bms_stateofcharge;
    packet.mavlink_version = 2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
}

/**
 * @brief Encode a heartbeat struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat)
{
    return mavlink_msg_heartbeat_pack(system_id, component_id, msg, heartbeat->type, heartbeat->master_status, heartbeat->subsystem_status, heartbeat->errors, heartbeat->bms_working, heartbeat->bms_voltage_mv, heartbeat->bms_current_ma, heartbeat->bms_stateofcharge);
}

/**
 * @brief Encode a heartbeat struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heartbeat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat)
{
    return mavlink_msg_heartbeat_pack_chan(system_id, component_id, chan, msg, heartbeat->type, heartbeat->master_status, heartbeat->subsystem_status, heartbeat->errors, heartbeat->bms_working, heartbeat->bms_voltage_mv, heartbeat->bms_current_ma, heartbeat->bms_stateofcharge);
}

/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param type Type of the heartbeat sender. defined in ANYMAL_SENDER_TYPES ENUM
 * @param master_status System status flag, comes from master
 * @param subsystem_status Subsystem's status that comes from master
 * @param errors Master module's specific errors
 * @param bms_working Specifies if bms is working
 * @param bms_voltage_mv Battery Voltage in mV
 * @param bms_current_ma Battery Current in mA
 * @param bms_stateofcharge Battery State of Charge %
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_heartbeat_send(mavlink_channel_t chan, uint8_t type, uint8_t master_status, uint16_t subsystem_status, uint16_t errors, uint8_t bms_working, uint16_t bms_voltage_mv, int16_t bms_current_ma, uint8_t bms_stateofcharge)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
    _mav_put_uint16_t(buf, 0, subsystem_status);
    _mav_put_uint16_t(buf, 2, errors);
    _mav_put_uint16_t(buf, 4, bms_voltage_mv);
    _mav_put_int16_t(buf, 6, bms_current_ma);
    _mav_put_uint8_t(buf, 8, type);
    _mav_put_uint8_t(buf, 9, master_status);
    _mav_put_uint8_t(buf, 10, bms_working);
    _mav_put_uint8_t(buf, 11, bms_stateofcharge);
    _mav_put_uint8_t(buf, 12, 2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#else
    mavlink_heartbeat_t packet;
    packet.subsystem_status = subsystem_status;
    packet.errors = errors;
    packet.bms_voltage_mv = bms_voltage_mv;
    packet.bms_current_ma = bms_current_ma;
    packet.type = type;
    packet.master_status = master_status;
    packet.bms_working = bms_working;
    packet.bms_stateofcharge = bms_stateofcharge;
    packet.mavlink_version = 2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)&packet, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#endif
}

/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_heartbeat_send_struct(mavlink_channel_t chan, const mavlink_heartbeat_t* heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_heartbeat_send(chan, heartbeat->type, heartbeat->master_status, heartbeat->subsystem_status, heartbeat->errors, heartbeat->bms_working, heartbeat->bms_voltage_mv, heartbeat->bms_current_ma, heartbeat->bms_stateofcharge);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)heartbeat, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#endif
}

#if MAVLINK_MSG_ID_HEARTBEAT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_heartbeat_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t type, uint8_t master_status, uint16_t subsystem_status, uint16_t errors, uint8_t bms_working, uint16_t bms_voltage_mv, int16_t bms_current_ma, uint8_t bms_stateofcharge)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, subsystem_status);
    _mav_put_uint16_t(buf, 2, errors);
    _mav_put_uint16_t(buf, 4, bms_voltage_mv);
    _mav_put_int16_t(buf, 6, bms_current_ma);
    _mav_put_uint8_t(buf, 8, type);
    _mav_put_uint8_t(buf, 9, master_status);
    _mav_put_uint8_t(buf, 10, bms_working);
    _mav_put_uint8_t(buf, 11, bms_stateofcharge);
    _mav_put_uint8_t(buf, 12, 2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#else
    mavlink_heartbeat_t *packet = (mavlink_heartbeat_t *)msgbuf;
    packet->subsystem_status = subsystem_status;
    packet->errors = errors;
    packet->bms_voltage_mv = bms_voltage_mv;
    packet->bms_current_ma = bms_current_ma;
    packet->type = type;
    packet->master_status = master_status;
    packet->bms_working = bms_working;
    packet->bms_stateofcharge = bms_stateofcharge;
    packet->mavlink_version = 2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)packet, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#endif
}
#endif

#endif

// MESSAGE HEARTBEAT UNPACKING


/**
 * @brief Get field type from heartbeat message
 *
 * @return Type of the heartbeat sender. defined in ANYMAL_SENDER_TYPES ENUM
 */
static inline uint8_t mavlink_msg_heartbeat_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field master_status from heartbeat message
 *
 * @return System status flag, comes from master
 */
static inline uint8_t mavlink_msg_heartbeat_get_master_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field subsystem_status from heartbeat message
 *
 * @return Subsystem's status that comes from master
 */
static inline uint16_t mavlink_msg_heartbeat_get_subsystem_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field errors from heartbeat message
 *
 * @return Master module's specific errors
 */
static inline uint16_t mavlink_msg_heartbeat_get_errors(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field bms_working from heartbeat message
 *
 * @return Specifies if bms is working
 */
static inline uint8_t mavlink_msg_heartbeat_get_bms_working(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field bms_voltage_mv from heartbeat message
 *
 * @return Battery Voltage in mV
 */
static inline uint16_t mavlink_msg_heartbeat_get_bms_voltage_mv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field bms_current_ma from heartbeat message
 *
 * @return Battery Current in mA
 */
static inline int16_t mavlink_msg_heartbeat_get_bms_current_ma(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field bms_stateofcharge from heartbeat message
 *
 * @return Battery State of Charge %
 */
static inline uint8_t mavlink_msg_heartbeat_get_bms_stateofcharge(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field mavlink_version from heartbeat message
 *
 * @return MAVLink version
 */
static inline uint8_t mavlink_msg_heartbeat_get_mavlink_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Decode a heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param heartbeat C-struct to decode the message contents into
 */
static inline void mavlink_msg_heartbeat_decode(const mavlink_message_t* msg, mavlink_heartbeat_t* heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    heartbeat->subsystem_status = mavlink_msg_heartbeat_get_subsystem_status(msg);
    heartbeat->errors = mavlink_msg_heartbeat_get_errors(msg);
    heartbeat->bms_voltage_mv = mavlink_msg_heartbeat_get_bms_voltage_mv(msg);
    heartbeat->bms_current_ma = mavlink_msg_heartbeat_get_bms_current_ma(msg);
    heartbeat->type = mavlink_msg_heartbeat_get_type(msg);
    heartbeat->master_status = mavlink_msg_heartbeat_get_master_status(msg);
    heartbeat->bms_working = mavlink_msg_heartbeat_get_bms_working(msg);
    heartbeat->bms_stateofcharge = mavlink_msg_heartbeat_get_bms_stateofcharge(msg);
    heartbeat->mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HEARTBEAT_LEN? msg->len : MAVLINK_MSG_ID_HEARTBEAT_LEN;
        memset(heartbeat, 0, MAVLINK_MSG_ID_HEARTBEAT_LEN);
    memcpy(heartbeat, _MAV_PAYLOAD(msg), len);
#endif
}
