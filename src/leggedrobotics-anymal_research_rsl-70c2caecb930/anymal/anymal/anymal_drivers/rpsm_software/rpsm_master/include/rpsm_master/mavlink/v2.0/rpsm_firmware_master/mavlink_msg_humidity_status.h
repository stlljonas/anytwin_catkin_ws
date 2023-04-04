#pragma once
// MESSAGE HUMIDITY_STATUS PACKING

#define MAVLINK_MSG_ID_HUMIDITY_STATUS 4

MAVPACKED(
typedef struct __mavlink_humidity_status_t {
 float ambient_temperature; /*< humidity sensor's temperature*/
 float ambient_humidity; /*< humidity sensor's relative humidity*/
 uint8_t humidity_working; /*< humidity sensor's operational state*/
}) mavlink_humidity_status_t;

#define MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN 9
#define MAVLINK_MSG_ID_HUMIDITY_STATUS_MIN_LEN 9
#define MAVLINK_MSG_ID_4_LEN 9
#define MAVLINK_MSG_ID_4_MIN_LEN 9

#define MAVLINK_MSG_ID_HUMIDITY_STATUS_CRC 80
#define MAVLINK_MSG_ID_4_CRC 80



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HUMIDITY_STATUS { \
    4, \
    "HUMIDITY_STATUS", \
    3, \
    {  { "ambient_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_humidity_status_t, ambient_temperature) }, \
         { "ambient_humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_humidity_status_t, ambient_humidity) }, \
         { "humidity_working", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_humidity_status_t, humidity_working) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HUMIDITY_STATUS { \
    "HUMIDITY_STATUS", \
    3, \
    {  { "ambient_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_humidity_status_t, ambient_temperature) }, \
         { "ambient_humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_humidity_status_t, ambient_humidity) }, \
         { "humidity_working", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_humidity_status_t, humidity_working) }, \
         } \
}
#endif

/**
 * @brief Pack a humidity_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ambient_temperature humidity sensor's temperature
 * @param ambient_humidity humidity sensor's relative humidity
 * @param humidity_working humidity sensor's operational state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_humidity_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float ambient_temperature, float ambient_humidity, uint8_t humidity_working)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN];
    _mav_put_float(buf, 0, ambient_temperature);
    _mav_put_float(buf, 4, ambient_humidity);
    _mav_put_uint8_t(buf, 8, humidity_working);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN);
#else
    mavlink_humidity_status_t packet;
    packet.ambient_temperature = ambient_temperature;
    packet.ambient_humidity = ambient_humidity;
    packet.humidity_working = humidity_working;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HUMIDITY_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HUMIDITY_STATUS_MIN_LEN, MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN, MAVLINK_MSG_ID_HUMIDITY_STATUS_CRC);
}

/**
 * @brief Pack a humidity_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ambient_temperature humidity sensor's temperature
 * @param ambient_humidity humidity sensor's relative humidity
 * @param humidity_working humidity sensor's operational state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_humidity_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float ambient_temperature,float ambient_humidity,uint8_t humidity_working)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN];
    _mav_put_float(buf, 0, ambient_temperature);
    _mav_put_float(buf, 4, ambient_humidity);
    _mav_put_uint8_t(buf, 8, humidity_working);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN);
#else
    mavlink_humidity_status_t packet;
    packet.ambient_temperature = ambient_temperature;
    packet.ambient_humidity = ambient_humidity;
    packet.humidity_working = humidity_working;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HUMIDITY_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HUMIDITY_STATUS_MIN_LEN, MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN, MAVLINK_MSG_ID_HUMIDITY_STATUS_CRC);
}

/**
 * @brief Encode a humidity_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param humidity_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_humidity_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_humidity_status_t* humidity_status)
{
    return mavlink_msg_humidity_status_pack(system_id, component_id, msg, humidity_status->ambient_temperature, humidity_status->ambient_humidity, humidity_status->humidity_working);
}

/**
 * @brief Encode a humidity_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param humidity_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_humidity_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_humidity_status_t* humidity_status)
{
    return mavlink_msg_humidity_status_pack_chan(system_id, component_id, chan, msg, humidity_status->ambient_temperature, humidity_status->ambient_humidity, humidity_status->humidity_working);
}

/**
 * @brief Send a humidity_status message
 * @param chan MAVLink channel to send the message
 *
 * @param ambient_temperature humidity sensor's temperature
 * @param ambient_humidity humidity sensor's relative humidity
 * @param humidity_working humidity sensor's operational state
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_humidity_status_send(mavlink_channel_t chan, float ambient_temperature, float ambient_humidity, uint8_t humidity_working)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN];
    _mav_put_float(buf, 0, ambient_temperature);
    _mav_put_float(buf, 4, ambient_humidity);
    _mav_put_uint8_t(buf, 8, humidity_working);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUMIDITY_STATUS, buf, MAVLINK_MSG_ID_HUMIDITY_STATUS_MIN_LEN, MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN, MAVLINK_MSG_ID_HUMIDITY_STATUS_CRC);
#else
    mavlink_humidity_status_t packet;
    packet.ambient_temperature = ambient_temperature;
    packet.ambient_humidity = ambient_humidity;
    packet.humidity_working = humidity_working;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUMIDITY_STATUS, (const char *)&packet, MAVLINK_MSG_ID_HUMIDITY_STATUS_MIN_LEN, MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN, MAVLINK_MSG_ID_HUMIDITY_STATUS_CRC);
#endif
}

/**
 * @brief Send a humidity_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_humidity_status_send_struct(mavlink_channel_t chan, const mavlink_humidity_status_t* humidity_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_humidity_status_send(chan, humidity_status->ambient_temperature, humidity_status->ambient_humidity, humidity_status->humidity_working);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUMIDITY_STATUS, (const char *)humidity_status, MAVLINK_MSG_ID_HUMIDITY_STATUS_MIN_LEN, MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN, MAVLINK_MSG_ID_HUMIDITY_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_humidity_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float ambient_temperature, float ambient_humidity, uint8_t humidity_working)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, ambient_temperature);
    _mav_put_float(buf, 4, ambient_humidity);
    _mav_put_uint8_t(buf, 8, humidity_working);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUMIDITY_STATUS, buf, MAVLINK_MSG_ID_HUMIDITY_STATUS_MIN_LEN, MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN, MAVLINK_MSG_ID_HUMIDITY_STATUS_CRC);
#else
    mavlink_humidity_status_t *packet = (mavlink_humidity_status_t *)msgbuf;
    packet->ambient_temperature = ambient_temperature;
    packet->ambient_humidity = ambient_humidity;
    packet->humidity_working = humidity_working;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HUMIDITY_STATUS, (const char *)packet, MAVLINK_MSG_ID_HUMIDITY_STATUS_MIN_LEN, MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN, MAVLINK_MSG_ID_HUMIDITY_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE HUMIDITY_STATUS UNPACKING


/**
 * @brief Get field ambient_temperature from humidity_status message
 *
 * @return humidity sensor's temperature
 */
static inline float mavlink_msg_humidity_status_get_ambient_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ambient_humidity from humidity_status message
 *
 * @return humidity sensor's relative humidity
 */
static inline float mavlink_msg_humidity_status_get_ambient_humidity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field humidity_working from humidity_status message
 *
 * @return humidity sensor's operational state
 */
static inline uint8_t mavlink_msg_humidity_status_get_humidity_working(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a humidity_status message into a struct
 *
 * @param msg The message to decode
 * @param humidity_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_humidity_status_decode(const mavlink_message_t* msg, mavlink_humidity_status_t* humidity_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    humidity_status->ambient_temperature = mavlink_msg_humidity_status_get_ambient_temperature(msg);
    humidity_status->ambient_humidity = mavlink_msg_humidity_status_get_ambient_humidity(msg);
    humidity_status->humidity_working = mavlink_msg_humidity_status_get_humidity_working(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN? msg->len : MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN;
        memset(humidity_status, 0, MAVLINK_MSG_ID_HUMIDITY_STATUS_LEN);
    memcpy(humidity_status, _MAV_PAYLOAD(msg), len);
#endif
}
