#pragma once
// MESSAGE BMS_CIRCUIT_STATUS PACKING

#define MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS 3

MAVPACKED(
typedef struct __mavlink_bms_circuit_status_t {
 uint32_t safetystatus; /*< BMS info*/
 uint32_t operationstatus; /*< BMS info*/
 uint16_t serialnumber; /*< BMS info*/
 uint16_t temperature; /*< BMS info*/
 uint16_t voltage; /*< BMS info*/
 int16_t current; /*< BMS info*/
 uint16_t batterystatus; /*< BMS info*/
 uint16_t cellvoltage1; /*< BMS info*/
 uint16_t cellvoltage2; /*< BMS info*/
 uint16_t cellvoltage3; /*< BMS info*/
 uint16_t cellvoltage4; /*< BMS info*/
 uint16_t cellvoltage5; /*< BMS info*/
 uint16_t cellvoltage6; /*< BMS info*/
 uint16_t cellvoltage7; /*< BMS info*/
 uint16_t cellvoltage8; /*< BMS info*/
 uint16_t cellvoltage9; /*< BMS info*/
 uint16_t cellvoltage10; /*< BMS info*/
 uint16_t cellvoltage11; /*< BMS info*/
 uint16_t cellvoltage12; /*< BMS info*/
 uint8_t stateofcharge; /*< BMS info*/
}) mavlink_bms_circuit_status_t;

#define MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN 43
#define MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_MIN_LEN 43
#define MAVLINK_MSG_ID_3_LEN 43
#define MAVLINK_MSG_ID_3_MIN_LEN 43

#define MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_CRC 73
#define MAVLINK_MSG_ID_3_CRC 73



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BMS_CIRCUIT_STATUS { \
    3, \
    "BMS_CIRCUIT_STATUS", \
    20, \
    {  { "safetystatus", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_bms_circuit_status_t, safetystatus) }, \
         { "operationstatus", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_bms_circuit_status_t, operationstatus) }, \
         { "serialnumber", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_bms_circuit_status_t, serialnumber) }, \
         { "temperature", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_bms_circuit_status_t, temperature) }, \
         { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_bms_circuit_status_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_bms_circuit_status_t, current) }, \
         { "batterystatus", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_bms_circuit_status_t, batterystatus) }, \
         { "cellvoltage1", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_bms_circuit_status_t, cellvoltage1) }, \
         { "cellvoltage2", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_bms_circuit_status_t, cellvoltage2) }, \
         { "cellvoltage3", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_bms_circuit_status_t, cellvoltage3) }, \
         { "cellvoltage4", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_bms_circuit_status_t, cellvoltage4) }, \
         { "cellvoltage5", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_bms_circuit_status_t, cellvoltage5) }, \
         { "cellvoltage6", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_bms_circuit_status_t, cellvoltage6) }, \
         { "cellvoltage7", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_bms_circuit_status_t, cellvoltage7) }, \
         { "cellvoltage8", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_bms_circuit_status_t, cellvoltage8) }, \
         { "cellvoltage9", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_bms_circuit_status_t, cellvoltage9) }, \
         { "cellvoltage10", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_bms_circuit_status_t, cellvoltage10) }, \
         { "cellvoltage11", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_bms_circuit_status_t, cellvoltage11) }, \
         { "cellvoltage12", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_bms_circuit_status_t, cellvoltage12) }, \
         { "stateofcharge", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_bms_circuit_status_t, stateofcharge) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BMS_CIRCUIT_STATUS { \
    "BMS_CIRCUIT_STATUS", \
    20, \
    {  { "safetystatus", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_bms_circuit_status_t, safetystatus) }, \
         { "operationstatus", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_bms_circuit_status_t, operationstatus) }, \
         { "serialnumber", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_bms_circuit_status_t, serialnumber) }, \
         { "temperature", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_bms_circuit_status_t, temperature) }, \
         { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_bms_circuit_status_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_bms_circuit_status_t, current) }, \
         { "batterystatus", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_bms_circuit_status_t, batterystatus) }, \
         { "cellvoltage1", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_bms_circuit_status_t, cellvoltage1) }, \
         { "cellvoltage2", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_bms_circuit_status_t, cellvoltage2) }, \
         { "cellvoltage3", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_bms_circuit_status_t, cellvoltage3) }, \
         { "cellvoltage4", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_bms_circuit_status_t, cellvoltage4) }, \
         { "cellvoltage5", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_bms_circuit_status_t, cellvoltage5) }, \
         { "cellvoltage6", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_bms_circuit_status_t, cellvoltage6) }, \
         { "cellvoltage7", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_bms_circuit_status_t, cellvoltage7) }, \
         { "cellvoltage8", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_bms_circuit_status_t, cellvoltage8) }, \
         { "cellvoltage9", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_bms_circuit_status_t, cellvoltage9) }, \
         { "cellvoltage10", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_bms_circuit_status_t, cellvoltage10) }, \
         { "cellvoltage11", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_bms_circuit_status_t, cellvoltage11) }, \
         { "cellvoltage12", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_bms_circuit_status_t, cellvoltage12) }, \
         { "stateofcharge", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_bms_circuit_status_t, stateofcharge) }, \
         } \
}
#endif

/**
 * @brief Pack a bms_circuit_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param serialnumber BMS info
 * @param temperature BMS info
 * @param voltage BMS info
 * @param current BMS info
 * @param stateofcharge BMS info
 * @param batterystatus BMS info
 * @param safetystatus BMS info
 * @param operationstatus BMS info
 * @param cellvoltage1 BMS info
 * @param cellvoltage2 BMS info
 * @param cellvoltage3 BMS info
 * @param cellvoltage4 BMS info
 * @param cellvoltage5 BMS info
 * @param cellvoltage6 BMS info
 * @param cellvoltage7 BMS info
 * @param cellvoltage8 BMS info
 * @param cellvoltage9 BMS info
 * @param cellvoltage10 BMS info
 * @param cellvoltage11 BMS info
 * @param cellvoltage12 BMS info
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bms_circuit_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t serialnumber, uint16_t temperature, uint16_t voltage, int16_t current, uint8_t stateofcharge, uint16_t batterystatus, uint32_t safetystatus, uint32_t operationstatus, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6, uint16_t cellvoltage7, uint16_t cellvoltage8, uint16_t cellvoltage9, uint16_t cellvoltage10, uint16_t cellvoltage11, uint16_t cellvoltage12)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, safetystatus);
    _mav_put_uint32_t(buf, 4, operationstatus);
    _mav_put_uint16_t(buf, 8, serialnumber);
    _mav_put_uint16_t(buf, 10, temperature);
    _mav_put_uint16_t(buf, 12, voltage);
    _mav_put_int16_t(buf, 14, current);
    _mav_put_uint16_t(buf, 16, batterystatus);
    _mav_put_uint16_t(buf, 18, cellvoltage1);
    _mav_put_uint16_t(buf, 20, cellvoltage2);
    _mav_put_uint16_t(buf, 22, cellvoltage3);
    _mav_put_uint16_t(buf, 24, cellvoltage4);
    _mav_put_uint16_t(buf, 26, cellvoltage5);
    _mav_put_uint16_t(buf, 28, cellvoltage6);
    _mav_put_uint16_t(buf, 30, cellvoltage7);
    _mav_put_uint16_t(buf, 32, cellvoltage8);
    _mav_put_uint16_t(buf, 34, cellvoltage9);
    _mav_put_uint16_t(buf, 36, cellvoltage10);
    _mav_put_uint16_t(buf, 38, cellvoltage11);
    _mav_put_uint16_t(buf, 40, cellvoltage12);
    _mav_put_uint8_t(buf, 42, stateofcharge);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN);
#else
    mavlink_bms_circuit_status_t packet;
    packet.safetystatus = safetystatus;
    packet.operationstatus = operationstatus;
    packet.serialnumber = serialnumber;
    packet.temperature = temperature;
    packet.voltage = voltage;
    packet.current = current;
    packet.batterystatus = batterystatus;
    packet.cellvoltage1 = cellvoltage1;
    packet.cellvoltage2 = cellvoltage2;
    packet.cellvoltage3 = cellvoltage3;
    packet.cellvoltage4 = cellvoltage4;
    packet.cellvoltage5 = cellvoltage5;
    packet.cellvoltage6 = cellvoltage6;
    packet.cellvoltage7 = cellvoltage7;
    packet.cellvoltage8 = cellvoltage8;
    packet.cellvoltage9 = cellvoltage9;
    packet.cellvoltage10 = cellvoltage10;
    packet.cellvoltage11 = cellvoltage11;
    packet.cellvoltage12 = cellvoltage12;
    packet.stateofcharge = stateofcharge;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_CRC);
}

/**
 * @brief Pack a bms_circuit_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param serialnumber BMS info
 * @param temperature BMS info
 * @param voltage BMS info
 * @param current BMS info
 * @param stateofcharge BMS info
 * @param batterystatus BMS info
 * @param safetystatus BMS info
 * @param operationstatus BMS info
 * @param cellvoltage1 BMS info
 * @param cellvoltage2 BMS info
 * @param cellvoltage3 BMS info
 * @param cellvoltage4 BMS info
 * @param cellvoltage5 BMS info
 * @param cellvoltage6 BMS info
 * @param cellvoltage7 BMS info
 * @param cellvoltage8 BMS info
 * @param cellvoltage9 BMS info
 * @param cellvoltage10 BMS info
 * @param cellvoltage11 BMS info
 * @param cellvoltage12 BMS info
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bms_circuit_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t serialnumber,uint16_t temperature,uint16_t voltage,int16_t current,uint8_t stateofcharge,uint16_t batterystatus,uint32_t safetystatus,uint32_t operationstatus,uint16_t cellvoltage1,uint16_t cellvoltage2,uint16_t cellvoltage3,uint16_t cellvoltage4,uint16_t cellvoltage5,uint16_t cellvoltage6,uint16_t cellvoltage7,uint16_t cellvoltage8,uint16_t cellvoltage9,uint16_t cellvoltage10,uint16_t cellvoltage11,uint16_t cellvoltage12)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, safetystatus);
    _mav_put_uint32_t(buf, 4, operationstatus);
    _mav_put_uint16_t(buf, 8, serialnumber);
    _mav_put_uint16_t(buf, 10, temperature);
    _mav_put_uint16_t(buf, 12, voltage);
    _mav_put_int16_t(buf, 14, current);
    _mav_put_uint16_t(buf, 16, batterystatus);
    _mav_put_uint16_t(buf, 18, cellvoltage1);
    _mav_put_uint16_t(buf, 20, cellvoltage2);
    _mav_put_uint16_t(buf, 22, cellvoltage3);
    _mav_put_uint16_t(buf, 24, cellvoltage4);
    _mav_put_uint16_t(buf, 26, cellvoltage5);
    _mav_put_uint16_t(buf, 28, cellvoltage6);
    _mav_put_uint16_t(buf, 30, cellvoltage7);
    _mav_put_uint16_t(buf, 32, cellvoltage8);
    _mav_put_uint16_t(buf, 34, cellvoltage9);
    _mav_put_uint16_t(buf, 36, cellvoltage10);
    _mav_put_uint16_t(buf, 38, cellvoltage11);
    _mav_put_uint16_t(buf, 40, cellvoltage12);
    _mav_put_uint8_t(buf, 42, stateofcharge);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN);
#else
    mavlink_bms_circuit_status_t packet;
    packet.safetystatus = safetystatus;
    packet.operationstatus = operationstatus;
    packet.serialnumber = serialnumber;
    packet.temperature = temperature;
    packet.voltage = voltage;
    packet.current = current;
    packet.batterystatus = batterystatus;
    packet.cellvoltage1 = cellvoltage1;
    packet.cellvoltage2 = cellvoltage2;
    packet.cellvoltage3 = cellvoltage3;
    packet.cellvoltage4 = cellvoltage4;
    packet.cellvoltage5 = cellvoltage5;
    packet.cellvoltage6 = cellvoltage6;
    packet.cellvoltage7 = cellvoltage7;
    packet.cellvoltage8 = cellvoltage8;
    packet.cellvoltage9 = cellvoltage9;
    packet.cellvoltage10 = cellvoltage10;
    packet.cellvoltage11 = cellvoltage11;
    packet.cellvoltage12 = cellvoltage12;
    packet.stateofcharge = stateofcharge;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_CRC);
}

/**
 * @brief Encode a bms_circuit_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param bms_circuit_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bms_circuit_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_bms_circuit_status_t* bms_circuit_status)
{
    return mavlink_msg_bms_circuit_status_pack(system_id, component_id, msg, bms_circuit_status->serialnumber, bms_circuit_status->temperature, bms_circuit_status->voltage, bms_circuit_status->current, bms_circuit_status->stateofcharge, bms_circuit_status->batterystatus, bms_circuit_status->safetystatus, bms_circuit_status->operationstatus, bms_circuit_status->cellvoltage1, bms_circuit_status->cellvoltage2, bms_circuit_status->cellvoltage3, bms_circuit_status->cellvoltage4, bms_circuit_status->cellvoltage5, bms_circuit_status->cellvoltage6, bms_circuit_status->cellvoltage7, bms_circuit_status->cellvoltage8, bms_circuit_status->cellvoltage9, bms_circuit_status->cellvoltage10, bms_circuit_status->cellvoltage11, bms_circuit_status->cellvoltage12);
}

/**
 * @brief Encode a bms_circuit_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param bms_circuit_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bms_circuit_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_bms_circuit_status_t* bms_circuit_status)
{
    return mavlink_msg_bms_circuit_status_pack_chan(system_id, component_id, chan, msg, bms_circuit_status->serialnumber, bms_circuit_status->temperature, bms_circuit_status->voltage, bms_circuit_status->current, bms_circuit_status->stateofcharge, bms_circuit_status->batterystatus, bms_circuit_status->safetystatus, bms_circuit_status->operationstatus, bms_circuit_status->cellvoltage1, bms_circuit_status->cellvoltage2, bms_circuit_status->cellvoltage3, bms_circuit_status->cellvoltage4, bms_circuit_status->cellvoltage5, bms_circuit_status->cellvoltage6, bms_circuit_status->cellvoltage7, bms_circuit_status->cellvoltage8, bms_circuit_status->cellvoltage9, bms_circuit_status->cellvoltage10, bms_circuit_status->cellvoltage11, bms_circuit_status->cellvoltage12);
}

/**
 * @brief Send a bms_circuit_status message
 * @param chan MAVLink channel to send the message
 *
 * @param serialnumber BMS info
 * @param temperature BMS info
 * @param voltage BMS info
 * @param current BMS info
 * @param stateofcharge BMS info
 * @param batterystatus BMS info
 * @param safetystatus BMS info
 * @param operationstatus BMS info
 * @param cellvoltage1 BMS info
 * @param cellvoltage2 BMS info
 * @param cellvoltage3 BMS info
 * @param cellvoltage4 BMS info
 * @param cellvoltage5 BMS info
 * @param cellvoltage6 BMS info
 * @param cellvoltage7 BMS info
 * @param cellvoltage8 BMS info
 * @param cellvoltage9 BMS info
 * @param cellvoltage10 BMS info
 * @param cellvoltage11 BMS info
 * @param cellvoltage12 BMS info
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_bms_circuit_status_send(mavlink_channel_t chan, uint16_t serialnumber, uint16_t temperature, uint16_t voltage, int16_t current, uint8_t stateofcharge, uint16_t batterystatus, uint32_t safetystatus, uint32_t operationstatus, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6, uint16_t cellvoltage7, uint16_t cellvoltage8, uint16_t cellvoltage9, uint16_t cellvoltage10, uint16_t cellvoltage11, uint16_t cellvoltage12)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, safetystatus);
    _mav_put_uint32_t(buf, 4, operationstatus);
    _mav_put_uint16_t(buf, 8, serialnumber);
    _mav_put_uint16_t(buf, 10, temperature);
    _mav_put_uint16_t(buf, 12, voltage);
    _mav_put_int16_t(buf, 14, current);
    _mav_put_uint16_t(buf, 16, batterystatus);
    _mav_put_uint16_t(buf, 18, cellvoltage1);
    _mav_put_uint16_t(buf, 20, cellvoltage2);
    _mav_put_uint16_t(buf, 22, cellvoltage3);
    _mav_put_uint16_t(buf, 24, cellvoltage4);
    _mav_put_uint16_t(buf, 26, cellvoltage5);
    _mav_put_uint16_t(buf, 28, cellvoltage6);
    _mav_put_uint16_t(buf, 30, cellvoltage7);
    _mav_put_uint16_t(buf, 32, cellvoltage8);
    _mav_put_uint16_t(buf, 34, cellvoltage9);
    _mav_put_uint16_t(buf, 36, cellvoltage10);
    _mav_put_uint16_t(buf, 38, cellvoltage11);
    _mav_put_uint16_t(buf, 40, cellvoltage12);
    _mav_put_uint8_t(buf, 42, stateofcharge);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS, buf, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_CRC);
#else
    mavlink_bms_circuit_status_t packet;
    packet.safetystatus = safetystatus;
    packet.operationstatus = operationstatus;
    packet.serialnumber = serialnumber;
    packet.temperature = temperature;
    packet.voltage = voltage;
    packet.current = current;
    packet.batterystatus = batterystatus;
    packet.cellvoltage1 = cellvoltage1;
    packet.cellvoltage2 = cellvoltage2;
    packet.cellvoltage3 = cellvoltage3;
    packet.cellvoltage4 = cellvoltage4;
    packet.cellvoltage5 = cellvoltage5;
    packet.cellvoltage6 = cellvoltage6;
    packet.cellvoltage7 = cellvoltage7;
    packet.cellvoltage8 = cellvoltage8;
    packet.cellvoltage9 = cellvoltage9;
    packet.cellvoltage10 = cellvoltage10;
    packet.cellvoltage11 = cellvoltage11;
    packet.cellvoltage12 = cellvoltage12;
    packet.stateofcharge = stateofcharge;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_CRC);
#endif
}

/**
 * @brief Send a bms_circuit_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_bms_circuit_status_send_struct(mavlink_channel_t chan, const mavlink_bms_circuit_status_t* bms_circuit_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_bms_circuit_status_send(chan, bms_circuit_status->serialnumber, bms_circuit_status->temperature, bms_circuit_status->voltage, bms_circuit_status->current, bms_circuit_status->stateofcharge, bms_circuit_status->batterystatus, bms_circuit_status->safetystatus, bms_circuit_status->operationstatus, bms_circuit_status->cellvoltage1, bms_circuit_status->cellvoltage2, bms_circuit_status->cellvoltage3, bms_circuit_status->cellvoltage4, bms_circuit_status->cellvoltage5, bms_circuit_status->cellvoltage6, bms_circuit_status->cellvoltage7, bms_circuit_status->cellvoltage8, bms_circuit_status->cellvoltage9, bms_circuit_status->cellvoltage10, bms_circuit_status->cellvoltage11, bms_circuit_status->cellvoltage12);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS, (const char *)bms_circuit_status, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_bms_circuit_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t serialnumber, uint16_t temperature, uint16_t voltage, int16_t current, uint8_t stateofcharge, uint16_t batterystatus, uint32_t safetystatus, uint32_t operationstatus, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6, uint16_t cellvoltage7, uint16_t cellvoltage8, uint16_t cellvoltage9, uint16_t cellvoltage10, uint16_t cellvoltage11, uint16_t cellvoltage12)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, safetystatus);
    _mav_put_uint32_t(buf, 4, operationstatus);
    _mav_put_uint16_t(buf, 8, serialnumber);
    _mav_put_uint16_t(buf, 10, temperature);
    _mav_put_uint16_t(buf, 12, voltage);
    _mav_put_int16_t(buf, 14, current);
    _mav_put_uint16_t(buf, 16, batterystatus);
    _mav_put_uint16_t(buf, 18, cellvoltage1);
    _mav_put_uint16_t(buf, 20, cellvoltage2);
    _mav_put_uint16_t(buf, 22, cellvoltage3);
    _mav_put_uint16_t(buf, 24, cellvoltage4);
    _mav_put_uint16_t(buf, 26, cellvoltage5);
    _mav_put_uint16_t(buf, 28, cellvoltage6);
    _mav_put_uint16_t(buf, 30, cellvoltage7);
    _mav_put_uint16_t(buf, 32, cellvoltage8);
    _mav_put_uint16_t(buf, 34, cellvoltage9);
    _mav_put_uint16_t(buf, 36, cellvoltage10);
    _mav_put_uint16_t(buf, 38, cellvoltage11);
    _mav_put_uint16_t(buf, 40, cellvoltage12);
    _mav_put_uint8_t(buf, 42, stateofcharge);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS, buf, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_CRC);
#else
    mavlink_bms_circuit_status_t *packet = (mavlink_bms_circuit_status_t *)msgbuf;
    packet->safetystatus = safetystatus;
    packet->operationstatus = operationstatus;
    packet->serialnumber = serialnumber;
    packet->temperature = temperature;
    packet->voltage = voltage;
    packet->current = current;
    packet->batterystatus = batterystatus;
    packet->cellvoltage1 = cellvoltage1;
    packet->cellvoltage2 = cellvoltage2;
    packet->cellvoltage3 = cellvoltage3;
    packet->cellvoltage4 = cellvoltage4;
    packet->cellvoltage5 = cellvoltage5;
    packet->cellvoltage6 = cellvoltage6;
    packet->cellvoltage7 = cellvoltage7;
    packet->cellvoltage8 = cellvoltage8;
    packet->cellvoltage9 = cellvoltage9;
    packet->cellvoltage10 = cellvoltage10;
    packet->cellvoltage11 = cellvoltage11;
    packet->cellvoltage12 = cellvoltage12;
    packet->stateofcharge = stateofcharge;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS, (const char *)packet, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_MIN_LEN, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE BMS_CIRCUIT_STATUS UNPACKING


/**
 * @brief Get field serialnumber from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_serialnumber(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field temperature from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field voltage from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field current from bms_circuit_status message
 *
 * @return BMS info
 */
static inline int16_t mavlink_msg_bms_circuit_status_get_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field stateofcharge from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint8_t mavlink_msg_bms_circuit_status_get_stateofcharge(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field batterystatus from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_batterystatus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field safetystatus from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint32_t mavlink_msg_bms_circuit_status_get_safetystatus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field operationstatus from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint32_t mavlink_msg_bms_circuit_status_get_operationstatus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field cellvoltage1 from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_cellvoltage1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field cellvoltage2 from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_cellvoltage2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field cellvoltage3 from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_cellvoltage3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field cellvoltage4 from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_cellvoltage4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field cellvoltage5 from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_cellvoltage5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field cellvoltage6 from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_cellvoltage6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field cellvoltage7 from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_cellvoltage7(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field cellvoltage8 from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_cellvoltage8(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field cellvoltage9 from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_cellvoltage9(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field cellvoltage10 from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_cellvoltage10(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field cellvoltage11 from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_cellvoltage11(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  38);
}

/**
 * @brief Get field cellvoltage12 from bms_circuit_status message
 *
 * @return BMS info
 */
static inline uint16_t mavlink_msg_bms_circuit_status_get_cellvoltage12(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Decode a bms_circuit_status message into a struct
 *
 * @param msg The message to decode
 * @param bms_circuit_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_bms_circuit_status_decode(const mavlink_message_t* msg, mavlink_bms_circuit_status_t* bms_circuit_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    bms_circuit_status->safetystatus = mavlink_msg_bms_circuit_status_get_safetystatus(msg);
    bms_circuit_status->operationstatus = mavlink_msg_bms_circuit_status_get_operationstatus(msg);
    bms_circuit_status->serialnumber = mavlink_msg_bms_circuit_status_get_serialnumber(msg);
    bms_circuit_status->temperature = mavlink_msg_bms_circuit_status_get_temperature(msg);
    bms_circuit_status->voltage = mavlink_msg_bms_circuit_status_get_voltage(msg);
    bms_circuit_status->current = mavlink_msg_bms_circuit_status_get_current(msg);
    bms_circuit_status->batterystatus = mavlink_msg_bms_circuit_status_get_batterystatus(msg);
    bms_circuit_status->cellvoltage1 = mavlink_msg_bms_circuit_status_get_cellvoltage1(msg);
    bms_circuit_status->cellvoltage2 = mavlink_msg_bms_circuit_status_get_cellvoltage2(msg);
    bms_circuit_status->cellvoltage3 = mavlink_msg_bms_circuit_status_get_cellvoltage3(msg);
    bms_circuit_status->cellvoltage4 = mavlink_msg_bms_circuit_status_get_cellvoltage4(msg);
    bms_circuit_status->cellvoltage5 = mavlink_msg_bms_circuit_status_get_cellvoltage5(msg);
    bms_circuit_status->cellvoltage6 = mavlink_msg_bms_circuit_status_get_cellvoltage6(msg);
    bms_circuit_status->cellvoltage7 = mavlink_msg_bms_circuit_status_get_cellvoltage7(msg);
    bms_circuit_status->cellvoltage8 = mavlink_msg_bms_circuit_status_get_cellvoltage8(msg);
    bms_circuit_status->cellvoltage9 = mavlink_msg_bms_circuit_status_get_cellvoltage9(msg);
    bms_circuit_status->cellvoltage10 = mavlink_msg_bms_circuit_status_get_cellvoltage10(msg);
    bms_circuit_status->cellvoltage11 = mavlink_msg_bms_circuit_status_get_cellvoltage11(msg);
    bms_circuit_status->cellvoltage12 = mavlink_msg_bms_circuit_status_get_cellvoltage12(msg);
    bms_circuit_status->stateofcharge = mavlink_msg_bms_circuit_status_get_stateofcharge(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN? msg->len : MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN;
        memset(bms_circuit_status, 0, MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_LEN);
    memcpy(bms_circuit_status, _MAV_PAYLOAD(msg), len);
#endif
}
