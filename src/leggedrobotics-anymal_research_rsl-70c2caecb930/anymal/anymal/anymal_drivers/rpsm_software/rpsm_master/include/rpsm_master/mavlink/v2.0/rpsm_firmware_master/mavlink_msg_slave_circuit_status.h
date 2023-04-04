#pragma once
// MESSAGE SLAVE_CIRCUIT_STATUS PACKING

#define MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS 2

MAVPACKED(
typedef struct __mavlink_slave_circuit_status_t {
 uint32_t voltage_mv_48V[4]; /*< Voltage on 48V bus of slave in mV*/
 int32_t current_ma_48V[4]; /*< Current on 48V bus of slave in mA*/
 uint32_t temperature_48V[4]; /*< Temperature on 48V bus of slave in mA*/
 uint8_t slave_id[4]; /*< slave ID of slave*/
 uint8_t estop[4]; /*< Temperature on 48V bus of slave in mA*/
}) mavlink_slave_circuit_status_t;

#define MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN 56
#define MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_MIN_LEN 56
#define MAVLINK_MSG_ID_2_LEN 56
#define MAVLINK_MSG_ID_2_MIN_LEN 56

#define MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_CRC 224
#define MAVLINK_MSG_ID_2_CRC 224

#define MAVLINK_MSG_SLAVE_CIRCUIT_STATUS_FIELD_VOLTAGE_MV_48V_LEN 4
#define MAVLINK_MSG_SLAVE_CIRCUIT_STATUS_FIELD_CURRENT_MA_48V_LEN 4
#define MAVLINK_MSG_SLAVE_CIRCUIT_STATUS_FIELD_TEMPERATURE_48V_LEN 4
#define MAVLINK_MSG_SLAVE_CIRCUIT_STATUS_FIELD_SLAVE_ID_LEN 4
#define MAVLINK_MSG_SLAVE_CIRCUIT_STATUS_FIELD_ESTOP_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SLAVE_CIRCUIT_STATUS { \
    2, \
    "SLAVE_CIRCUIT_STATUS", \
    5, \
    {  { "voltage_mv_48V", NULL, MAVLINK_TYPE_UINT32_T, 4, 0, offsetof(mavlink_slave_circuit_status_t, voltage_mv_48V) }, \
         { "current_ma_48V", NULL, MAVLINK_TYPE_INT32_T, 4, 16, offsetof(mavlink_slave_circuit_status_t, current_ma_48V) }, \
         { "temperature_48V", NULL, MAVLINK_TYPE_UINT32_T, 4, 32, offsetof(mavlink_slave_circuit_status_t, temperature_48V) }, \
         { "slave_id", NULL, MAVLINK_TYPE_UINT8_T, 4, 48, offsetof(mavlink_slave_circuit_status_t, slave_id) }, \
         { "estop", NULL, MAVLINK_TYPE_UINT8_T, 4, 52, offsetof(mavlink_slave_circuit_status_t, estop) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SLAVE_CIRCUIT_STATUS { \
    "SLAVE_CIRCUIT_STATUS", \
    5, \
    {  { "voltage_mv_48V", NULL, MAVLINK_TYPE_UINT32_T, 4, 0, offsetof(mavlink_slave_circuit_status_t, voltage_mv_48V) }, \
         { "current_ma_48V", NULL, MAVLINK_TYPE_INT32_T, 4, 16, offsetof(mavlink_slave_circuit_status_t, current_ma_48V) }, \
         { "temperature_48V", NULL, MAVLINK_TYPE_UINT32_T, 4, 32, offsetof(mavlink_slave_circuit_status_t, temperature_48V) }, \
         { "slave_id", NULL, MAVLINK_TYPE_UINT8_T, 4, 48, offsetof(mavlink_slave_circuit_status_t, slave_id) }, \
         { "estop", NULL, MAVLINK_TYPE_UINT8_T, 4, 52, offsetof(mavlink_slave_circuit_status_t, estop) }, \
         } \
}
#endif

/**
 * @brief Pack a slave_circuit_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param slave_id slave ID of slave
 * @param voltage_mv_48V Voltage on 48V bus of slave in mV
 * @param current_ma_48V Current on 48V bus of slave in mA
 * @param temperature_48V Temperature on 48V bus of slave in mA
 * @param estop Temperature on 48V bus of slave in mA
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slave_circuit_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint8_t *slave_id, const uint32_t *voltage_mv_48V, const int32_t *current_ma_48V, const uint32_t *temperature_48V, const uint8_t *estop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN];

    _mav_put_uint32_t_array(buf, 0, voltage_mv_48V, 4);
    _mav_put_int32_t_array(buf, 16, current_ma_48V, 4);
    _mav_put_uint32_t_array(buf, 32, temperature_48V, 4);
    _mav_put_uint8_t_array(buf, 48, slave_id, 4);
    _mav_put_uint8_t_array(buf, 52, estop, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN);
#else
    mavlink_slave_circuit_status_t packet;

    mav_array_memcpy(packet.voltage_mv_48V, voltage_mv_48V, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.current_ma_48V, current_ma_48V, sizeof(int32_t)*4);
    mav_array_memcpy(packet.temperature_48V, temperature_48V, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.slave_id, slave_id, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.estop, estop, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_CRC);
}

/**
 * @brief Pack a slave_circuit_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param slave_id slave ID of slave
 * @param voltage_mv_48V Voltage on 48V bus of slave in mV
 * @param current_ma_48V Current on 48V bus of slave in mA
 * @param temperature_48V Temperature on 48V bus of slave in mA
 * @param estop Temperature on 48V bus of slave in mA
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slave_circuit_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint8_t *slave_id,const uint32_t *voltage_mv_48V,const int32_t *current_ma_48V,const uint32_t *temperature_48V,const uint8_t *estop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN];

    _mav_put_uint32_t_array(buf, 0, voltage_mv_48V, 4);
    _mav_put_int32_t_array(buf, 16, current_ma_48V, 4);
    _mav_put_uint32_t_array(buf, 32, temperature_48V, 4);
    _mav_put_uint8_t_array(buf, 48, slave_id, 4);
    _mav_put_uint8_t_array(buf, 52, estop, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN);
#else
    mavlink_slave_circuit_status_t packet;

    mav_array_memcpy(packet.voltage_mv_48V, voltage_mv_48V, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.current_ma_48V, current_ma_48V, sizeof(int32_t)*4);
    mav_array_memcpy(packet.temperature_48V, temperature_48V, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.slave_id, slave_id, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.estop, estop, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_CRC);
}

/**
 * @brief Encode a slave_circuit_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param slave_circuit_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_slave_circuit_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_slave_circuit_status_t* slave_circuit_status)
{
    return mavlink_msg_slave_circuit_status_pack(system_id, component_id, msg, slave_circuit_status->slave_id, slave_circuit_status->voltage_mv_48V, slave_circuit_status->current_ma_48V, slave_circuit_status->temperature_48V, slave_circuit_status->estop);
}

/**
 * @brief Encode a slave_circuit_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param slave_circuit_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_slave_circuit_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_slave_circuit_status_t* slave_circuit_status)
{
    return mavlink_msg_slave_circuit_status_pack_chan(system_id, component_id, chan, msg, slave_circuit_status->slave_id, slave_circuit_status->voltage_mv_48V, slave_circuit_status->current_ma_48V, slave_circuit_status->temperature_48V, slave_circuit_status->estop);
}

/**
 * @brief Send a slave_circuit_status message
 * @param chan MAVLink channel to send the message
 *
 * @param slave_id slave ID of slave
 * @param voltage_mv_48V Voltage on 48V bus of slave in mV
 * @param current_ma_48V Current on 48V bus of slave in mA
 * @param temperature_48V Temperature on 48V bus of slave in mA
 * @param estop Temperature on 48V bus of slave in mA
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_slave_circuit_status_send(mavlink_channel_t chan, const uint8_t *slave_id, const uint32_t *voltage_mv_48V, const int32_t *current_ma_48V, const uint32_t *temperature_48V, const uint8_t *estop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN];

    _mav_put_uint32_t_array(buf, 0, voltage_mv_48V, 4);
    _mav_put_int32_t_array(buf, 16, current_ma_48V, 4);
    _mav_put_uint32_t_array(buf, 32, temperature_48V, 4);
    _mav_put_uint8_t_array(buf, 48, slave_id, 4);
    _mav_put_uint8_t_array(buf, 52, estop, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS, buf, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_CRC);
#else
    mavlink_slave_circuit_status_t packet;

    mav_array_memcpy(packet.voltage_mv_48V, voltage_mv_48V, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.current_ma_48V, current_ma_48V, sizeof(int32_t)*4);
    mav_array_memcpy(packet.temperature_48V, temperature_48V, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.slave_id, slave_id, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.estop, estop, sizeof(uint8_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_CRC);
#endif
}

/**
 * @brief Send a slave_circuit_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_slave_circuit_status_send_struct(mavlink_channel_t chan, const mavlink_slave_circuit_status_t* slave_circuit_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_slave_circuit_status_send(chan, slave_circuit_status->slave_id, slave_circuit_status->voltage_mv_48V, slave_circuit_status->current_ma_48V, slave_circuit_status->temperature_48V, slave_circuit_status->estop);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS, (const char *)slave_circuit_status, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_slave_circuit_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint8_t *slave_id, const uint32_t *voltage_mv_48V, const int32_t *current_ma_48V, const uint32_t *temperature_48V, const uint8_t *estop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint32_t_array(buf, 0, voltage_mv_48V, 4);
    _mav_put_int32_t_array(buf, 16, current_ma_48V, 4);
    _mav_put_uint32_t_array(buf, 32, temperature_48V, 4);
    _mav_put_uint8_t_array(buf, 48, slave_id, 4);
    _mav_put_uint8_t_array(buf, 52, estop, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS, buf, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_CRC);
#else
    mavlink_slave_circuit_status_t *packet = (mavlink_slave_circuit_status_t *)msgbuf;

    mav_array_memcpy(packet->voltage_mv_48V, voltage_mv_48V, sizeof(uint32_t)*4);
    mav_array_memcpy(packet->current_ma_48V, current_ma_48V, sizeof(int32_t)*4);
    mav_array_memcpy(packet->temperature_48V, temperature_48V, sizeof(uint32_t)*4);
    mav_array_memcpy(packet->slave_id, slave_id, sizeof(uint8_t)*4);
    mav_array_memcpy(packet->estop, estop, sizeof(uint8_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS, (const char *)packet, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE SLAVE_CIRCUIT_STATUS UNPACKING


/**
 * @brief Get field slave_id from slave_circuit_status message
 *
 * @return slave ID of slave
 */
static inline uint16_t mavlink_msg_slave_circuit_status_get_slave_id(const mavlink_message_t* msg, uint8_t *slave_id)
{
    return _MAV_RETURN_uint8_t_array(msg, slave_id, 4,  48);
}

/**
 * @brief Get field voltage_mv_48V from slave_circuit_status message
 *
 * @return Voltage on 48V bus of slave in mV
 */
static inline uint16_t mavlink_msg_slave_circuit_status_get_voltage_mv_48V(const mavlink_message_t* msg, uint32_t *voltage_mv_48V)
{
    return _MAV_RETURN_uint32_t_array(msg, voltage_mv_48V, 4,  0);
}

/**
 * @brief Get field current_ma_48V from slave_circuit_status message
 *
 * @return Current on 48V bus of slave in mA
 */
static inline uint16_t mavlink_msg_slave_circuit_status_get_current_ma_48V(const mavlink_message_t* msg, int32_t *current_ma_48V)
{
    return _MAV_RETURN_int32_t_array(msg, current_ma_48V, 4,  16);
}

/**
 * @brief Get field temperature_48V from slave_circuit_status message
 *
 * @return Temperature on 48V bus of slave in mA
 */
static inline uint16_t mavlink_msg_slave_circuit_status_get_temperature_48V(const mavlink_message_t* msg, uint32_t *temperature_48V)
{
    return _MAV_RETURN_uint32_t_array(msg, temperature_48V, 4,  32);
}

/**
 * @brief Get field estop from slave_circuit_status message
 *
 * @return Temperature on 48V bus of slave in mA
 */
static inline uint16_t mavlink_msg_slave_circuit_status_get_estop(const mavlink_message_t* msg, uint8_t *estop)
{
    return _MAV_RETURN_uint8_t_array(msg, estop, 4,  52);
}

/**
 * @brief Decode a slave_circuit_status message into a struct
 *
 * @param msg The message to decode
 * @param slave_circuit_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_slave_circuit_status_decode(const mavlink_message_t* msg, mavlink_slave_circuit_status_t* slave_circuit_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_slave_circuit_status_get_voltage_mv_48V(msg, slave_circuit_status->voltage_mv_48V);
    mavlink_msg_slave_circuit_status_get_current_ma_48V(msg, slave_circuit_status->current_ma_48V);
    mavlink_msg_slave_circuit_status_get_temperature_48V(msg, slave_circuit_status->temperature_48V);
    mavlink_msg_slave_circuit_status_get_slave_id(msg, slave_circuit_status->slave_id);
    mavlink_msg_slave_circuit_status_get_estop(msg, slave_circuit_status->estop);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN? msg->len : MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN;
        memset(slave_circuit_status, 0, MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_LEN);
    memcpy(slave_circuit_status, _MAV_PAYLOAD(msg), len);
#endif
}
