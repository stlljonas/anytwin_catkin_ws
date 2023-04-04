#pragma once
// MESSAGE UAVCAN_NETWORK_STATUS PACKING

#define MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS 5

MAVPACKED(
typedef struct __mavlink_uavcan_network_status_t {
 uint32_t master_up_time_s; /*< master errorflags*/
 uint32_t slave_uptime_s[16]; /*< slave uptime*/
 uint8_t nr_of_slaves; /*< number of slaves in network*/
 uint8_t master_id; /*< ID of Master*/
 uint8_t master_mode; /*< mode of Master*/
 uint8_t master_health; /*< Health of Master*/
 uint8_t master_errorflags; /*< master errorflags*/
 uint8_t slave_ids[16]; /*< slave ids*/
 uint8_t slave_mode[16]; /*< slave modes*/
 uint8_t slave_health[16]; /*< slave health*/
 uint8_t slave_errorflags[16]; /*< slave errorflags*/
}) mavlink_uavcan_network_status_t;

#define MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN 137
#define MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_MIN_LEN 137
#define MAVLINK_MSG_ID_5_LEN 137
#define MAVLINK_MSG_ID_5_MIN_LEN 137

#define MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_CRC 161
#define MAVLINK_MSG_ID_5_CRC 161

#define MAVLINK_MSG_UAVCAN_NETWORK_STATUS_FIELD_SLAVE_UPTIME_S_LEN 16
#define MAVLINK_MSG_UAVCAN_NETWORK_STATUS_FIELD_SLAVE_IDS_LEN 16
#define MAVLINK_MSG_UAVCAN_NETWORK_STATUS_FIELD_SLAVE_MODE_LEN 16
#define MAVLINK_MSG_UAVCAN_NETWORK_STATUS_FIELD_SLAVE_HEALTH_LEN 16
#define MAVLINK_MSG_UAVCAN_NETWORK_STATUS_FIELD_SLAVE_ERRORFLAGS_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAVCAN_NETWORK_STATUS { \
    5, \
    "UAVCAN_NETWORK_STATUS", \
    11, \
    {  { "master_up_time_s", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_uavcan_network_status_t, master_up_time_s) }, \
         { "slave_uptime_s", NULL, MAVLINK_TYPE_UINT32_T, 16, 4, offsetof(mavlink_uavcan_network_status_t, slave_uptime_s) }, \
         { "nr_of_slaves", NULL, MAVLINK_TYPE_UINT8_T, 0, 68, offsetof(mavlink_uavcan_network_status_t, nr_of_slaves) }, \
         { "master_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 69, offsetof(mavlink_uavcan_network_status_t, master_id) }, \
         { "master_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 70, offsetof(mavlink_uavcan_network_status_t, master_mode) }, \
         { "master_health", NULL, MAVLINK_TYPE_UINT8_T, 0, 71, offsetof(mavlink_uavcan_network_status_t, master_health) }, \
         { "master_errorflags", NULL, MAVLINK_TYPE_UINT8_T, 0, 72, offsetof(mavlink_uavcan_network_status_t, master_errorflags) }, \
         { "slave_ids", NULL, MAVLINK_TYPE_UINT8_T, 16, 73, offsetof(mavlink_uavcan_network_status_t, slave_ids) }, \
         { "slave_mode", NULL, MAVLINK_TYPE_UINT8_T, 16, 89, offsetof(mavlink_uavcan_network_status_t, slave_mode) }, \
         { "slave_health", NULL, MAVLINK_TYPE_UINT8_T, 16, 105, offsetof(mavlink_uavcan_network_status_t, slave_health) }, \
         { "slave_errorflags", NULL, MAVLINK_TYPE_UINT8_T, 16, 121, offsetof(mavlink_uavcan_network_status_t, slave_errorflags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAVCAN_NETWORK_STATUS { \
    "UAVCAN_NETWORK_STATUS", \
    11, \
    {  { "master_up_time_s", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_uavcan_network_status_t, master_up_time_s) }, \
         { "slave_uptime_s", NULL, MAVLINK_TYPE_UINT32_T, 16, 4, offsetof(mavlink_uavcan_network_status_t, slave_uptime_s) }, \
         { "nr_of_slaves", NULL, MAVLINK_TYPE_UINT8_T, 0, 68, offsetof(mavlink_uavcan_network_status_t, nr_of_slaves) }, \
         { "master_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 69, offsetof(mavlink_uavcan_network_status_t, master_id) }, \
         { "master_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 70, offsetof(mavlink_uavcan_network_status_t, master_mode) }, \
         { "master_health", NULL, MAVLINK_TYPE_UINT8_T, 0, 71, offsetof(mavlink_uavcan_network_status_t, master_health) }, \
         { "master_errorflags", NULL, MAVLINK_TYPE_UINT8_T, 0, 72, offsetof(mavlink_uavcan_network_status_t, master_errorflags) }, \
         { "slave_ids", NULL, MAVLINK_TYPE_UINT8_T, 16, 73, offsetof(mavlink_uavcan_network_status_t, slave_ids) }, \
         { "slave_mode", NULL, MAVLINK_TYPE_UINT8_T, 16, 89, offsetof(mavlink_uavcan_network_status_t, slave_mode) }, \
         { "slave_health", NULL, MAVLINK_TYPE_UINT8_T, 16, 105, offsetof(mavlink_uavcan_network_status_t, slave_health) }, \
         { "slave_errorflags", NULL, MAVLINK_TYPE_UINT8_T, 16, 121, offsetof(mavlink_uavcan_network_status_t, slave_errorflags) }, \
         } \
}
#endif

/**
 * @brief Pack a uavcan_network_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param nr_of_slaves number of slaves in network
 * @param master_id ID of Master
 * @param master_mode mode of Master
 * @param master_health Health of Master
 * @param master_errorflags master errorflags
 * @param master_up_time_s master errorflags
 * @param slave_ids slave ids
 * @param slave_mode slave modes
 * @param slave_health slave health
 * @param slave_errorflags slave errorflags
 * @param slave_uptime_s slave uptime
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavcan_network_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t nr_of_slaves, uint8_t master_id, uint8_t master_mode, uint8_t master_health, uint8_t master_errorflags, uint32_t master_up_time_s, const uint8_t *slave_ids, const uint8_t *slave_mode, const uint8_t *slave_health, const uint8_t *slave_errorflags, const uint32_t *slave_uptime_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, master_up_time_s);
    _mav_put_uint8_t(buf, 68, nr_of_slaves);
    _mav_put_uint8_t(buf, 69, master_id);
    _mav_put_uint8_t(buf, 70, master_mode);
    _mav_put_uint8_t(buf, 71, master_health);
    _mav_put_uint8_t(buf, 72, master_errorflags);
    _mav_put_uint32_t_array(buf, 4, slave_uptime_s, 16);
    _mav_put_uint8_t_array(buf, 73, slave_ids, 16);
    _mav_put_uint8_t_array(buf, 89, slave_mode, 16);
    _mav_put_uint8_t_array(buf, 105, slave_health, 16);
    _mav_put_uint8_t_array(buf, 121, slave_errorflags, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN);
#else
    mavlink_uavcan_network_status_t packet;
    packet.master_up_time_s = master_up_time_s;
    packet.nr_of_slaves = nr_of_slaves;
    packet.master_id = master_id;
    packet.master_mode = master_mode;
    packet.master_health = master_health;
    packet.master_errorflags = master_errorflags;
    mav_array_memcpy(packet.slave_uptime_s, slave_uptime_s, sizeof(uint32_t)*16);
    mav_array_memcpy(packet.slave_ids, slave_ids, sizeof(uint8_t)*16);
    mav_array_memcpy(packet.slave_mode, slave_mode, sizeof(uint8_t)*16);
    mav_array_memcpy(packet.slave_health, slave_health, sizeof(uint8_t)*16);
    mav_array_memcpy(packet.slave_errorflags, slave_errorflags, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_CRC);
}

/**
 * @brief Pack a uavcan_network_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nr_of_slaves number of slaves in network
 * @param master_id ID of Master
 * @param master_mode mode of Master
 * @param master_health Health of Master
 * @param master_errorflags master errorflags
 * @param master_up_time_s master errorflags
 * @param slave_ids slave ids
 * @param slave_mode slave modes
 * @param slave_health slave health
 * @param slave_errorflags slave errorflags
 * @param slave_uptime_s slave uptime
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavcan_network_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t nr_of_slaves,uint8_t master_id,uint8_t master_mode,uint8_t master_health,uint8_t master_errorflags,uint32_t master_up_time_s,const uint8_t *slave_ids,const uint8_t *slave_mode,const uint8_t *slave_health,const uint8_t *slave_errorflags,const uint32_t *slave_uptime_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, master_up_time_s);
    _mav_put_uint8_t(buf, 68, nr_of_slaves);
    _mav_put_uint8_t(buf, 69, master_id);
    _mav_put_uint8_t(buf, 70, master_mode);
    _mav_put_uint8_t(buf, 71, master_health);
    _mav_put_uint8_t(buf, 72, master_errorflags);
    _mav_put_uint32_t_array(buf, 4, slave_uptime_s, 16);
    _mav_put_uint8_t_array(buf, 73, slave_ids, 16);
    _mav_put_uint8_t_array(buf, 89, slave_mode, 16);
    _mav_put_uint8_t_array(buf, 105, slave_health, 16);
    _mav_put_uint8_t_array(buf, 121, slave_errorflags, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN);
#else
    mavlink_uavcan_network_status_t packet;
    packet.master_up_time_s = master_up_time_s;
    packet.nr_of_slaves = nr_of_slaves;
    packet.master_id = master_id;
    packet.master_mode = master_mode;
    packet.master_health = master_health;
    packet.master_errorflags = master_errorflags;
    mav_array_memcpy(packet.slave_uptime_s, slave_uptime_s, sizeof(uint32_t)*16);
    mav_array_memcpy(packet.slave_ids, slave_ids, sizeof(uint8_t)*16);
    mav_array_memcpy(packet.slave_mode, slave_mode, sizeof(uint8_t)*16);
    mav_array_memcpy(packet.slave_health, slave_health, sizeof(uint8_t)*16);
    mav_array_memcpy(packet.slave_errorflags, slave_errorflags, sizeof(uint8_t)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_CRC);
}

/**
 * @brief Encode a uavcan_network_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uavcan_network_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavcan_network_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uavcan_network_status_t* uavcan_network_status)
{
    return mavlink_msg_uavcan_network_status_pack(system_id, component_id, msg, uavcan_network_status->nr_of_slaves, uavcan_network_status->master_id, uavcan_network_status->master_mode, uavcan_network_status->master_health, uavcan_network_status->master_errorflags, uavcan_network_status->master_up_time_s, uavcan_network_status->slave_ids, uavcan_network_status->slave_mode, uavcan_network_status->slave_health, uavcan_network_status->slave_errorflags, uavcan_network_status->slave_uptime_s);
}

/**
 * @brief Encode a uavcan_network_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uavcan_network_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavcan_network_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uavcan_network_status_t* uavcan_network_status)
{
    return mavlink_msg_uavcan_network_status_pack_chan(system_id, component_id, chan, msg, uavcan_network_status->nr_of_slaves, uavcan_network_status->master_id, uavcan_network_status->master_mode, uavcan_network_status->master_health, uavcan_network_status->master_errorflags, uavcan_network_status->master_up_time_s, uavcan_network_status->slave_ids, uavcan_network_status->slave_mode, uavcan_network_status->slave_health, uavcan_network_status->slave_errorflags, uavcan_network_status->slave_uptime_s);
}

/**
 * @brief Send a uavcan_network_status message
 * @param chan MAVLink channel to send the message
 *
 * @param nr_of_slaves number of slaves in network
 * @param master_id ID of Master
 * @param master_mode mode of Master
 * @param master_health Health of Master
 * @param master_errorflags master errorflags
 * @param master_up_time_s master errorflags
 * @param slave_ids slave ids
 * @param slave_mode slave modes
 * @param slave_health slave health
 * @param slave_errorflags slave errorflags
 * @param slave_uptime_s slave uptime
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uavcan_network_status_send(mavlink_channel_t chan, uint8_t nr_of_slaves, uint8_t master_id, uint8_t master_mode, uint8_t master_health, uint8_t master_errorflags, uint32_t master_up_time_s, const uint8_t *slave_ids, const uint8_t *slave_mode, const uint8_t *slave_health, const uint8_t *slave_errorflags, const uint32_t *slave_uptime_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, master_up_time_s);
    _mav_put_uint8_t(buf, 68, nr_of_slaves);
    _mav_put_uint8_t(buf, 69, master_id);
    _mav_put_uint8_t(buf, 70, master_mode);
    _mav_put_uint8_t(buf, 71, master_health);
    _mav_put_uint8_t(buf, 72, master_errorflags);
    _mav_put_uint32_t_array(buf, 4, slave_uptime_s, 16);
    _mav_put_uint8_t_array(buf, 73, slave_ids, 16);
    _mav_put_uint8_t_array(buf, 89, slave_mode, 16);
    _mav_put_uint8_t_array(buf, 105, slave_health, 16);
    _mav_put_uint8_t_array(buf, 121, slave_errorflags, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS, buf, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_CRC);
#else
    mavlink_uavcan_network_status_t packet;
    packet.master_up_time_s = master_up_time_s;
    packet.nr_of_slaves = nr_of_slaves;
    packet.master_id = master_id;
    packet.master_mode = master_mode;
    packet.master_health = master_health;
    packet.master_errorflags = master_errorflags;
    mav_array_memcpy(packet.slave_uptime_s, slave_uptime_s, sizeof(uint32_t)*16);
    mav_array_memcpy(packet.slave_ids, slave_ids, sizeof(uint8_t)*16);
    mav_array_memcpy(packet.slave_mode, slave_mode, sizeof(uint8_t)*16);
    mav_array_memcpy(packet.slave_health, slave_health, sizeof(uint8_t)*16);
    mav_array_memcpy(packet.slave_errorflags, slave_errorflags, sizeof(uint8_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS, (const char *)&packet, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_CRC);
#endif
}

/**
 * @brief Send a uavcan_network_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uavcan_network_status_send_struct(mavlink_channel_t chan, const mavlink_uavcan_network_status_t* uavcan_network_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavcan_network_status_send(chan, uavcan_network_status->nr_of_slaves, uavcan_network_status->master_id, uavcan_network_status->master_mode, uavcan_network_status->master_health, uavcan_network_status->master_errorflags, uavcan_network_status->master_up_time_s, uavcan_network_status->slave_ids, uavcan_network_status->slave_mode, uavcan_network_status->slave_health, uavcan_network_status->slave_errorflags, uavcan_network_status->slave_uptime_s);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS, (const char *)uavcan_network_status, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uavcan_network_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t nr_of_slaves, uint8_t master_id, uint8_t master_mode, uint8_t master_health, uint8_t master_errorflags, uint32_t master_up_time_s, const uint8_t *slave_ids, const uint8_t *slave_mode, const uint8_t *slave_health, const uint8_t *slave_errorflags, const uint32_t *slave_uptime_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, master_up_time_s);
    _mav_put_uint8_t(buf, 68, nr_of_slaves);
    _mav_put_uint8_t(buf, 69, master_id);
    _mav_put_uint8_t(buf, 70, master_mode);
    _mav_put_uint8_t(buf, 71, master_health);
    _mav_put_uint8_t(buf, 72, master_errorflags);
    _mav_put_uint32_t_array(buf, 4, slave_uptime_s, 16);
    _mav_put_uint8_t_array(buf, 73, slave_ids, 16);
    _mav_put_uint8_t_array(buf, 89, slave_mode, 16);
    _mav_put_uint8_t_array(buf, 105, slave_health, 16);
    _mav_put_uint8_t_array(buf, 121, slave_errorflags, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS, buf, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_CRC);
#else
    mavlink_uavcan_network_status_t *packet = (mavlink_uavcan_network_status_t *)msgbuf;
    packet->master_up_time_s = master_up_time_s;
    packet->nr_of_slaves = nr_of_slaves;
    packet->master_id = master_id;
    packet->master_mode = master_mode;
    packet->master_health = master_health;
    packet->master_errorflags = master_errorflags;
    mav_array_memcpy(packet->slave_uptime_s, slave_uptime_s, sizeof(uint32_t)*16);
    mav_array_memcpy(packet->slave_ids, slave_ids, sizeof(uint8_t)*16);
    mav_array_memcpy(packet->slave_mode, slave_mode, sizeof(uint8_t)*16);
    mav_array_memcpy(packet->slave_health, slave_health, sizeof(uint8_t)*16);
    mav_array_memcpy(packet->slave_errorflags, slave_errorflags, sizeof(uint8_t)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS, (const char *)packet, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE UAVCAN_NETWORK_STATUS UNPACKING


/**
 * @brief Get field nr_of_slaves from uavcan_network_status message
 *
 * @return number of slaves in network
 */
static inline uint8_t mavlink_msg_uavcan_network_status_get_nr_of_slaves(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  68);
}

/**
 * @brief Get field master_id from uavcan_network_status message
 *
 * @return ID of Master
 */
static inline uint8_t mavlink_msg_uavcan_network_status_get_master_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  69);
}

/**
 * @brief Get field master_mode from uavcan_network_status message
 *
 * @return mode of Master
 */
static inline uint8_t mavlink_msg_uavcan_network_status_get_master_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  70);
}

/**
 * @brief Get field master_health from uavcan_network_status message
 *
 * @return Health of Master
 */
static inline uint8_t mavlink_msg_uavcan_network_status_get_master_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  71);
}

/**
 * @brief Get field master_errorflags from uavcan_network_status message
 *
 * @return master errorflags
 */
static inline uint8_t mavlink_msg_uavcan_network_status_get_master_errorflags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  72);
}

/**
 * @brief Get field master_up_time_s from uavcan_network_status message
 *
 * @return master errorflags
 */
static inline uint32_t mavlink_msg_uavcan_network_status_get_master_up_time_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field slave_ids from uavcan_network_status message
 *
 * @return slave ids
 */
static inline uint16_t mavlink_msg_uavcan_network_status_get_slave_ids(const mavlink_message_t* msg, uint8_t *slave_ids)
{
    return _MAV_RETURN_uint8_t_array(msg, slave_ids, 16,  73);
}

/**
 * @brief Get field slave_mode from uavcan_network_status message
 *
 * @return slave modes
 */
static inline uint16_t mavlink_msg_uavcan_network_status_get_slave_mode(const mavlink_message_t* msg, uint8_t *slave_mode)
{
    return _MAV_RETURN_uint8_t_array(msg, slave_mode, 16,  89);
}

/**
 * @brief Get field slave_health from uavcan_network_status message
 *
 * @return slave health
 */
static inline uint16_t mavlink_msg_uavcan_network_status_get_slave_health(const mavlink_message_t* msg, uint8_t *slave_health)
{
    return _MAV_RETURN_uint8_t_array(msg, slave_health, 16,  105);
}

/**
 * @brief Get field slave_errorflags from uavcan_network_status message
 *
 * @return slave errorflags
 */
static inline uint16_t mavlink_msg_uavcan_network_status_get_slave_errorflags(const mavlink_message_t* msg, uint8_t *slave_errorflags)
{
    return _MAV_RETURN_uint8_t_array(msg, slave_errorflags, 16,  121);
}

/**
 * @brief Get field slave_uptime_s from uavcan_network_status message
 *
 * @return slave uptime
 */
static inline uint16_t mavlink_msg_uavcan_network_status_get_slave_uptime_s(const mavlink_message_t* msg, uint32_t *slave_uptime_s)
{
    return _MAV_RETURN_uint32_t_array(msg, slave_uptime_s, 16,  4);
}

/**
 * @brief Decode a uavcan_network_status message into a struct
 *
 * @param msg The message to decode
 * @param uavcan_network_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_uavcan_network_status_decode(const mavlink_message_t* msg, mavlink_uavcan_network_status_t* uavcan_network_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uavcan_network_status->master_up_time_s = mavlink_msg_uavcan_network_status_get_master_up_time_s(msg);
    mavlink_msg_uavcan_network_status_get_slave_uptime_s(msg, uavcan_network_status->slave_uptime_s);
    uavcan_network_status->nr_of_slaves = mavlink_msg_uavcan_network_status_get_nr_of_slaves(msg);
    uavcan_network_status->master_id = mavlink_msg_uavcan_network_status_get_master_id(msg);
    uavcan_network_status->master_mode = mavlink_msg_uavcan_network_status_get_master_mode(msg);
    uavcan_network_status->master_health = mavlink_msg_uavcan_network_status_get_master_health(msg);
    uavcan_network_status->master_errorflags = mavlink_msg_uavcan_network_status_get_master_errorflags(msg);
    mavlink_msg_uavcan_network_status_get_slave_ids(msg, uavcan_network_status->slave_ids);
    mavlink_msg_uavcan_network_status_get_slave_mode(msg, uavcan_network_status->slave_mode);
    mavlink_msg_uavcan_network_status_get_slave_health(msg, uavcan_network_status->slave_health);
    mavlink_msg_uavcan_network_status_get_slave_errorflags(msg, uavcan_network_status->slave_errorflags);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN? msg->len : MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN;
        memset(uavcan_network_status, 0, MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_LEN);
    memcpy(uavcan_network_status, _MAV_PAYLOAD(msg), len);
#endif
}
