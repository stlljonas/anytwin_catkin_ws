#pragma once
// MESSAGE MASTER_CIRCUIT_STATUS PACKING

#define MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS 1

MAVPACKED(
typedef struct __mavlink_master_circuit_status_t {
 uint32_t voltage_mv_5V; /*< Voltage on 5V bus of slave in mV*/
 uint32_t voltage_mv_12V; /*< Voltage on 12V bus of slave in mV*/
 uint32_t voltage_mv_15V; /*< Voltage on 15V bus of slave in mV*/
 int32_t current_ma_5V; /*< Current on 5V bus of slave in mA*/
 int32_t current_ma_12V; /*< Current on 12V bus of slave in mA*/
 int32_t current_ma_15V; /*< Current on 15V bus of slave in mA*/
 int32_t temperature_5V; /*< Temperature on 5V bus of slave in C*/
 int32_t temperature_12V; /*< Temperature on 12V bus of slave in C*/
 int32_t temperature_15V; /*< Temperature on 15V bus of slave in C*/
 int8_t operational_5V; /*< is ADC operational*/
 int8_t operational_12V; /*< is ADC operational*/
 int8_t operational_15V; /*< is ADC operational*/
}) mavlink_master_circuit_status_t;

#define MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN 39
#define MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_MIN_LEN 39
#define MAVLINK_MSG_ID_1_LEN 39
#define MAVLINK_MSG_ID_1_MIN_LEN 39

#define MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_CRC 195
#define MAVLINK_MSG_ID_1_CRC 195



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MASTER_CIRCUIT_STATUS { \
    1, \
    "MASTER_CIRCUIT_STATUS", \
    12, \
    {  { "voltage_mv_5V", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_master_circuit_status_t, voltage_mv_5V) }, \
         { "voltage_mv_12V", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_master_circuit_status_t, voltage_mv_12V) }, \
         { "voltage_mv_15V", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_master_circuit_status_t, voltage_mv_15V) }, \
         { "current_ma_5V", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_master_circuit_status_t, current_ma_5V) }, \
         { "current_ma_12V", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_master_circuit_status_t, current_ma_12V) }, \
         { "current_ma_15V", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_master_circuit_status_t, current_ma_15V) }, \
         { "temperature_5V", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_master_circuit_status_t, temperature_5V) }, \
         { "temperature_12V", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_master_circuit_status_t, temperature_12V) }, \
         { "temperature_15V", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_master_circuit_status_t, temperature_15V) }, \
         { "operational_5V", NULL, MAVLINK_TYPE_INT8_T, 0, 36, offsetof(mavlink_master_circuit_status_t, operational_5V) }, \
         { "operational_12V", NULL, MAVLINK_TYPE_INT8_T, 0, 37, offsetof(mavlink_master_circuit_status_t, operational_12V) }, \
         { "operational_15V", NULL, MAVLINK_TYPE_INT8_T, 0, 38, offsetof(mavlink_master_circuit_status_t, operational_15V) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MASTER_CIRCUIT_STATUS { \
    "MASTER_CIRCUIT_STATUS", \
    12, \
    {  { "voltage_mv_5V", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_master_circuit_status_t, voltage_mv_5V) }, \
         { "voltage_mv_12V", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_master_circuit_status_t, voltage_mv_12V) }, \
         { "voltage_mv_15V", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_master_circuit_status_t, voltage_mv_15V) }, \
         { "current_ma_5V", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_master_circuit_status_t, current_ma_5V) }, \
         { "current_ma_12V", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_master_circuit_status_t, current_ma_12V) }, \
         { "current_ma_15V", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_master_circuit_status_t, current_ma_15V) }, \
         { "temperature_5V", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_master_circuit_status_t, temperature_5V) }, \
         { "temperature_12V", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_master_circuit_status_t, temperature_12V) }, \
         { "temperature_15V", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_master_circuit_status_t, temperature_15V) }, \
         { "operational_5V", NULL, MAVLINK_TYPE_INT8_T, 0, 36, offsetof(mavlink_master_circuit_status_t, operational_5V) }, \
         { "operational_12V", NULL, MAVLINK_TYPE_INT8_T, 0, 37, offsetof(mavlink_master_circuit_status_t, operational_12V) }, \
         { "operational_15V", NULL, MAVLINK_TYPE_INT8_T, 0, 38, offsetof(mavlink_master_circuit_status_t, operational_15V) }, \
         } \
}
#endif

/**
 * @brief Pack a master_circuit_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param voltage_mv_5V Voltage on 5V bus of slave in mV
 * @param voltage_mv_12V Voltage on 12V bus of slave in mV
 * @param voltage_mv_15V Voltage on 15V bus of slave in mV
 * @param current_ma_5V Current on 5V bus of slave in mA
 * @param current_ma_12V Current on 12V bus of slave in mA
 * @param current_ma_15V Current on 15V bus of slave in mA
 * @param temperature_5V Temperature on 5V bus of slave in C
 * @param temperature_12V Temperature on 12V bus of slave in C
 * @param temperature_15V Temperature on 15V bus of slave in C
 * @param operational_5V is ADC operational
 * @param operational_12V is ADC operational
 * @param operational_15V is ADC operational
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_master_circuit_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t voltage_mv_5V, uint32_t voltage_mv_12V, uint32_t voltage_mv_15V, int32_t current_ma_5V, int32_t current_ma_12V, int32_t current_ma_15V, int32_t temperature_5V, int32_t temperature_12V, int32_t temperature_15V, int8_t operational_5V, int8_t operational_12V, int8_t operational_15V)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, voltage_mv_5V);
    _mav_put_uint32_t(buf, 4, voltage_mv_12V);
    _mav_put_uint32_t(buf, 8, voltage_mv_15V);
    _mav_put_int32_t(buf, 12, current_ma_5V);
    _mav_put_int32_t(buf, 16, current_ma_12V);
    _mav_put_int32_t(buf, 20, current_ma_15V);
    _mav_put_int32_t(buf, 24, temperature_5V);
    _mav_put_int32_t(buf, 28, temperature_12V);
    _mav_put_int32_t(buf, 32, temperature_15V);
    _mav_put_int8_t(buf, 36, operational_5V);
    _mav_put_int8_t(buf, 37, operational_12V);
    _mav_put_int8_t(buf, 38, operational_15V);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN);
#else
    mavlink_master_circuit_status_t packet;
    packet.voltage_mv_5V = voltage_mv_5V;
    packet.voltage_mv_12V = voltage_mv_12V;
    packet.voltage_mv_15V = voltage_mv_15V;
    packet.current_ma_5V = current_ma_5V;
    packet.current_ma_12V = current_ma_12V;
    packet.current_ma_15V = current_ma_15V;
    packet.temperature_5V = temperature_5V;
    packet.temperature_12V = temperature_12V;
    packet.temperature_15V = temperature_15V;
    packet.operational_5V = operational_5V;
    packet.operational_12V = operational_12V;
    packet.operational_15V = operational_15V;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_CRC);
}

/**
 * @brief Pack a master_circuit_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param voltage_mv_5V Voltage on 5V bus of slave in mV
 * @param voltage_mv_12V Voltage on 12V bus of slave in mV
 * @param voltage_mv_15V Voltage on 15V bus of slave in mV
 * @param current_ma_5V Current on 5V bus of slave in mA
 * @param current_ma_12V Current on 12V bus of slave in mA
 * @param current_ma_15V Current on 15V bus of slave in mA
 * @param temperature_5V Temperature on 5V bus of slave in C
 * @param temperature_12V Temperature on 12V bus of slave in C
 * @param temperature_15V Temperature on 15V bus of slave in C
 * @param operational_5V is ADC operational
 * @param operational_12V is ADC operational
 * @param operational_15V is ADC operational
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_master_circuit_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t voltage_mv_5V,uint32_t voltage_mv_12V,uint32_t voltage_mv_15V,int32_t current_ma_5V,int32_t current_ma_12V,int32_t current_ma_15V,int32_t temperature_5V,int32_t temperature_12V,int32_t temperature_15V,int8_t operational_5V,int8_t operational_12V,int8_t operational_15V)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, voltage_mv_5V);
    _mav_put_uint32_t(buf, 4, voltage_mv_12V);
    _mav_put_uint32_t(buf, 8, voltage_mv_15V);
    _mav_put_int32_t(buf, 12, current_ma_5V);
    _mav_put_int32_t(buf, 16, current_ma_12V);
    _mav_put_int32_t(buf, 20, current_ma_15V);
    _mav_put_int32_t(buf, 24, temperature_5V);
    _mav_put_int32_t(buf, 28, temperature_12V);
    _mav_put_int32_t(buf, 32, temperature_15V);
    _mav_put_int8_t(buf, 36, operational_5V);
    _mav_put_int8_t(buf, 37, operational_12V);
    _mav_put_int8_t(buf, 38, operational_15V);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN);
#else
    mavlink_master_circuit_status_t packet;
    packet.voltage_mv_5V = voltage_mv_5V;
    packet.voltage_mv_12V = voltage_mv_12V;
    packet.voltage_mv_15V = voltage_mv_15V;
    packet.current_ma_5V = current_ma_5V;
    packet.current_ma_12V = current_ma_12V;
    packet.current_ma_15V = current_ma_15V;
    packet.temperature_5V = temperature_5V;
    packet.temperature_12V = temperature_12V;
    packet.temperature_15V = temperature_15V;
    packet.operational_5V = operational_5V;
    packet.operational_12V = operational_12V;
    packet.operational_15V = operational_15V;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_CRC);
}

/**
 * @brief Encode a master_circuit_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param master_circuit_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_master_circuit_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_master_circuit_status_t* master_circuit_status)
{
    return mavlink_msg_master_circuit_status_pack(system_id, component_id, msg, master_circuit_status->voltage_mv_5V, master_circuit_status->voltage_mv_12V, master_circuit_status->voltage_mv_15V, master_circuit_status->current_ma_5V, master_circuit_status->current_ma_12V, master_circuit_status->current_ma_15V, master_circuit_status->temperature_5V, master_circuit_status->temperature_12V, master_circuit_status->temperature_15V, master_circuit_status->operational_5V, master_circuit_status->operational_12V, master_circuit_status->operational_15V);
}

/**
 * @brief Encode a master_circuit_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param master_circuit_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_master_circuit_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_master_circuit_status_t* master_circuit_status)
{
    return mavlink_msg_master_circuit_status_pack_chan(system_id, component_id, chan, msg, master_circuit_status->voltage_mv_5V, master_circuit_status->voltage_mv_12V, master_circuit_status->voltage_mv_15V, master_circuit_status->current_ma_5V, master_circuit_status->current_ma_12V, master_circuit_status->current_ma_15V, master_circuit_status->temperature_5V, master_circuit_status->temperature_12V, master_circuit_status->temperature_15V, master_circuit_status->operational_5V, master_circuit_status->operational_12V, master_circuit_status->operational_15V);
}

/**
 * @brief Send a master_circuit_status message
 * @param chan MAVLink channel to send the message
 *
 * @param voltage_mv_5V Voltage on 5V bus of slave in mV
 * @param voltage_mv_12V Voltage on 12V bus of slave in mV
 * @param voltage_mv_15V Voltage on 15V bus of slave in mV
 * @param current_ma_5V Current on 5V bus of slave in mA
 * @param current_ma_12V Current on 12V bus of slave in mA
 * @param current_ma_15V Current on 15V bus of slave in mA
 * @param temperature_5V Temperature on 5V bus of slave in C
 * @param temperature_12V Temperature on 12V bus of slave in C
 * @param temperature_15V Temperature on 15V bus of slave in C
 * @param operational_5V is ADC operational
 * @param operational_12V is ADC operational
 * @param operational_15V is ADC operational
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_master_circuit_status_send(mavlink_channel_t chan, uint32_t voltage_mv_5V, uint32_t voltage_mv_12V, uint32_t voltage_mv_15V, int32_t current_ma_5V, int32_t current_ma_12V, int32_t current_ma_15V, int32_t temperature_5V, int32_t temperature_12V, int32_t temperature_15V, int8_t operational_5V, int8_t operational_12V, int8_t operational_15V)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, voltage_mv_5V);
    _mav_put_uint32_t(buf, 4, voltage_mv_12V);
    _mav_put_uint32_t(buf, 8, voltage_mv_15V);
    _mav_put_int32_t(buf, 12, current_ma_5V);
    _mav_put_int32_t(buf, 16, current_ma_12V);
    _mav_put_int32_t(buf, 20, current_ma_15V);
    _mav_put_int32_t(buf, 24, temperature_5V);
    _mav_put_int32_t(buf, 28, temperature_12V);
    _mav_put_int32_t(buf, 32, temperature_15V);
    _mav_put_int8_t(buf, 36, operational_5V);
    _mav_put_int8_t(buf, 37, operational_12V);
    _mav_put_int8_t(buf, 38, operational_15V);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS, buf, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_CRC);
#else
    mavlink_master_circuit_status_t packet;
    packet.voltage_mv_5V = voltage_mv_5V;
    packet.voltage_mv_12V = voltage_mv_12V;
    packet.voltage_mv_15V = voltage_mv_15V;
    packet.current_ma_5V = current_ma_5V;
    packet.current_ma_12V = current_ma_12V;
    packet.current_ma_15V = current_ma_15V;
    packet.temperature_5V = temperature_5V;
    packet.temperature_12V = temperature_12V;
    packet.temperature_15V = temperature_15V;
    packet.operational_5V = operational_5V;
    packet.operational_12V = operational_12V;
    packet.operational_15V = operational_15V;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_CRC);
#endif
}

/**
 * @brief Send a master_circuit_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_master_circuit_status_send_struct(mavlink_channel_t chan, const mavlink_master_circuit_status_t* master_circuit_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_master_circuit_status_send(chan, master_circuit_status->voltage_mv_5V, master_circuit_status->voltage_mv_12V, master_circuit_status->voltage_mv_15V, master_circuit_status->current_ma_5V, master_circuit_status->current_ma_12V, master_circuit_status->current_ma_15V, master_circuit_status->temperature_5V, master_circuit_status->temperature_12V, master_circuit_status->temperature_15V, master_circuit_status->operational_5V, master_circuit_status->operational_12V, master_circuit_status->operational_15V);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS, (const char *)master_circuit_status, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_master_circuit_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t voltage_mv_5V, uint32_t voltage_mv_12V, uint32_t voltage_mv_15V, int32_t current_ma_5V, int32_t current_ma_12V, int32_t current_ma_15V, int32_t temperature_5V, int32_t temperature_12V, int32_t temperature_15V, int8_t operational_5V, int8_t operational_12V, int8_t operational_15V)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, voltage_mv_5V);
    _mav_put_uint32_t(buf, 4, voltage_mv_12V);
    _mav_put_uint32_t(buf, 8, voltage_mv_15V);
    _mav_put_int32_t(buf, 12, current_ma_5V);
    _mav_put_int32_t(buf, 16, current_ma_12V);
    _mav_put_int32_t(buf, 20, current_ma_15V);
    _mav_put_int32_t(buf, 24, temperature_5V);
    _mav_put_int32_t(buf, 28, temperature_12V);
    _mav_put_int32_t(buf, 32, temperature_15V);
    _mav_put_int8_t(buf, 36, operational_5V);
    _mav_put_int8_t(buf, 37, operational_12V);
    _mav_put_int8_t(buf, 38, operational_15V);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS, buf, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_CRC);
#else
    mavlink_master_circuit_status_t *packet = (mavlink_master_circuit_status_t *)msgbuf;
    packet->voltage_mv_5V = voltage_mv_5V;
    packet->voltage_mv_12V = voltage_mv_12V;
    packet->voltage_mv_15V = voltage_mv_15V;
    packet->current_ma_5V = current_ma_5V;
    packet->current_ma_12V = current_ma_12V;
    packet->current_ma_15V = current_ma_15V;
    packet->temperature_5V = temperature_5V;
    packet->temperature_12V = temperature_12V;
    packet->temperature_15V = temperature_15V;
    packet->operational_5V = operational_5V;
    packet->operational_12V = operational_12V;
    packet->operational_15V = operational_15V;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS, (const char *)packet, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE MASTER_CIRCUIT_STATUS UNPACKING


/**
 * @brief Get field voltage_mv_5V from master_circuit_status message
 *
 * @return Voltage on 5V bus of slave in mV
 */
static inline uint32_t mavlink_msg_master_circuit_status_get_voltage_mv_5V(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field voltage_mv_12V from master_circuit_status message
 *
 * @return Voltage on 12V bus of slave in mV
 */
static inline uint32_t mavlink_msg_master_circuit_status_get_voltage_mv_12V(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field voltage_mv_15V from master_circuit_status message
 *
 * @return Voltage on 15V bus of slave in mV
 */
static inline uint32_t mavlink_msg_master_circuit_status_get_voltage_mv_15V(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field current_ma_5V from master_circuit_status message
 *
 * @return Current on 5V bus of slave in mA
 */
static inline int32_t mavlink_msg_master_circuit_status_get_current_ma_5V(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field current_ma_12V from master_circuit_status message
 *
 * @return Current on 12V bus of slave in mA
 */
static inline int32_t mavlink_msg_master_circuit_status_get_current_ma_12V(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field current_ma_15V from master_circuit_status message
 *
 * @return Current on 15V bus of slave in mA
 */
static inline int32_t mavlink_msg_master_circuit_status_get_current_ma_15V(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field temperature_5V from master_circuit_status message
 *
 * @return Temperature on 5V bus of slave in C
 */
static inline int32_t mavlink_msg_master_circuit_status_get_temperature_5V(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field temperature_12V from master_circuit_status message
 *
 * @return Temperature on 12V bus of slave in C
 */
static inline int32_t mavlink_msg_master_circuit_status_get_temperature_12V(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field temperature_15V from master_circuit_status message
 *
 * @return Temperature on 15V bus of slave in C
 */
static inline int32_t mavlink_msg_master_circuit_status_get_temperature_15V(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field operational_5V from master_circuit_status message
 *
 * @return is ADC operational
 */
static inline int8_t mavlink_msg_master_circuit_status_get_operational_5V(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  36);
}

/**
 * @brief Get field operational_12V from master_circuit_status message
 *
 * @return is ADC operational
 */
static inline int8_t mavlink_msg_master_circuit_status_get_operational_12V(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  37);
}

/**
 * @brief Get field operational_15V from master_circuit_status message
 *
 * @return is ADC operational
 */
static inline int8_t mavlink_msg_master_circuit_status_get_operational_15V(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  38);
}

/**
 * @brief Decode a master_circuit_status message into a struct
 *
 * @param msg The message to decode
 * @param master_circuit_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_master_circuit_status_decode(const mavlink_message_t* msg, mavlink_master_circuit_status_t* master_circuit_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    master_circuit_status->voltage_mv_5V = mavlink_msg_master_circuit_status_get_voltage_mv_5V(msg);
    master_circuit_status->voltage_mv_12V = mavlink_msg_master_circuit_status_get_voltage_mv_12V(msg);
    master_circuit_status->voltage_mv_15V = mavlink_msg_master_circuit_status_get_voltage_mv_15V(msg);
    master_circuit_status->current_ma_5V = mavlink_msg_master_circuit_status_get_current_ma_5V(msg);
    master_circuit_status->current_ma_12V = mavlink_msg_master_circuit_status_get_current_ma_12V(msg);
    master_circuit_status->current_ma_15V = mavlink_msg_master_circuit_status_get_current_ma_15V(msg);
    master_circuit_status->temperature_5V = mavlink_msg_master_circuit_status_get_temperature_5V(msg);
    master_circuit_status->temperature_12V = mavlink_msg_master_circuit_status_get_temperature_12V(msg);
    master_circuit_status->temperature_15V = mavlink_msg_master_circuit_status_get_temperature_15V(msg);
    master_circuit_status->operational_5V = mavlink_msg_master_circuit_status_get_operational_5V(msg);
    master_circuit_status->operational_12V = mavlink_msg_master_circuit_status_get_operational_12V(msg);
    master_circuit_status->operational_15V = mavlink_msg_master_circuit_status_get_operational_15V(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN? msg->len : MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN;
        memset(master_circuit_status, 0, MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_LEN);
    memcpy(master_circuit_status, _MAV_PAYLOAD(msg), len);
#endif
}
