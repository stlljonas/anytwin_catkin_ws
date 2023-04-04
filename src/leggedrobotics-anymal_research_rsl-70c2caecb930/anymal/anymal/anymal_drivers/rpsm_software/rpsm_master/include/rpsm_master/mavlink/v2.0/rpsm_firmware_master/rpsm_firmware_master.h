/** @file
 *  @brief MAVLink comm protocol generated from rpsm_firmware_master.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_RPSM_FIRMWARE_MASTER_H
#define MAVLINK_RPSM_FIRMWARE_MASTER_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_RPSM_FIRMWARE_MASTER.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 202, 13, 0, 0, 0}, {1, 195, 39, 0, 0, 0}, {2, 224, 56, 0, 0, 0}, {3, 73, 43, 0, 0, 0}, {4, 80, 9, 0, 0, 0}, {5, 161, 137, 0, 0, 0}, {6, 94, 12, 0, 0, 0}, {7, 190, 12, 0, 0, 0}, {8, 50, 65, 0, 0, 0}, {20, 136, 3, 0, 0, 0}, {21, 63, 7, 0, 0, 0}, {22, 201, 2, 0, 0, 0}, {23, 84, 254, 3, 1, 2}, {30, 150, 1, 0, 0, 0}, {31, 152, 1, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_RPSM_FIRMWARE_MASTER

// ENUM DEFINITIONS


/** @brief Specifies what device is talking to us. This identifies the individual model. */
#ifndef HAVE_ENUM_ANYMAL_MAVLNK_SENDER_TYPES
#define HAVE_ENUM_ANYMAL_MAVLNK_SENDER_TYPES
typedef enum ANYMAL_MAVLNK_SENDER_TYPES
{
   ANYMAL_MAVLINK_MASTER=0, /* rpsm_firmware_master sender | */
   ANYMAL_MAVLINK_OPERATOR_PC=1, /* External Operator PC | */
   ANYMAL_MAVILNK_LOCOMOTION_PC=2, /* Onboard Lowlevel PC | */
   ANYMAL_MAVLNK_SENDER_TYPES_ENUM_END=3, /*  | */
} ANYMAL_MAVLNK_SENDER_TYPES;
#endif

/** @brief  */
#ifndef HAVE_ENUM_ANYMAL_STATE
#define HAVE_ENUM_ANYMAL_STATE
typedef enum ANYMAL_STATE
{
   ANYMAL_STATE_NORMAL=0, /* Uninitialized system, state is unknown. | */
   ANYMAL_STATE_BOOT=1, /* System is booting up. | */
   ANYMAL_STATE_SOFT_EMERGENCY=2, /* System is calibrating and not flight-ready. | */
   ANYMAL_STATE_HARD_EMERGENCY=3, /* System is calibrating and not flight-ready. | */
   ANYMAL_STATE_POWEROFF=4, /* System just initialized its power-down sequence, will shut down now. | */
   ANYMAL_STATE_ENUM_END=5, /*  | */
} ANYMAL_STATE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_SLAVE_ENABLE_CMD
#define HAVE_ENUM_SLAVE_ENABLE_CMD
typedef enum SLAVE_ENABLE_CMD
{
   SLAVE_PIN_SETLOW=0, /* set Pin low. | */
   SLAVE_PIN_SETHIGH=1, /* set Pin high. | */
   SLAVE_PIN_TOGGLE=2, /* toggle pin. | */
   SLAVE_ENABLE_CMD_ENUM_END=3, /*  | */
} SLAVE_ENABLE_CMD;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MASTER_MAVLINK_NOTIFICATION
#define HAVE_ENUM_MASTER_MAVLINK_NOTIFICATION
typedef enum MASTER_MAVLINK_NOTIFICATION
{
   LEVEL_DEBUG=0, /* Notification level | */
   LEVEL_INFO=1, /* Notification level | */
   LEVEL_WARN=2, /* Notification level | */
   LEVEL_ERROR=3, /* Notification level | */
   LEVEL_FATAL=4, /* Notification level | */
   LEVEL_SHUTDOWN=5, /* Notification level | */
   LEVEL_ESTOP=6, /* Notification level | */
   MASTER_MAVLINK_NOTIFICATION_ENUM_END=7, /*  | */
} MASTER_MAVLINK_NOTIFICATION;
#endif

/** @brief  */
#ifndef HAVE_ENUM_PX4GPIO_PIN
#define HAVE_ENUM_PX4GPIO_PIN
typedef enum PX4GPIO_PIN
{
   PX4GPIO_LOW=0, /* Set gpio to low | */
   PX4GPIO_HIGH=1, /* Set gpio to high | */
   PX4GPIO_NA=2, /* Do not change pin | */
   PX4GPIO_PIN_ENUM_END=3, /*  | */
} PX4GPIO_PIN;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_heartbeat.h"
#include "./mavlink_msg_master_circuit_status.h"
#include "./mavlink_msg_slave_circuit_status.h"
#include "./mavlink_msg_bms_circuit_status.h"
#include "./mavlink_msg_humidity_status.h"
#include "./mavlink_msg_uavcan_network_status.h"
#include "./mavlink_msg_uavcan_network_statistics.h"
#include "./mavlink_msg_wireless_statistics.h"
#include "./mavlink_msg_master_mavlink_notification.h"
#include "./mavlink_msg_get_command.h"
#include "./mavlink_msg_px4gpio_set_cmd.h"
#include "./mavlink_msg_slave_enable_cmd.h"
#include "./mavlink_msg_file_transfer_protocol.h"
#include "./mavlink_msg_ack.h"
#include "./mavlink_msg_get_command_nack.h"

// base include


#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_MASTER_CIRCUIT_STATUS, MAVLINK_MESSAGE_INFO_SLAVE_CIRCUIT_STATUS, MAVLINK_MESSAGE_INFO_BMS_CIRCUIT_STATUS, MAVLINK_MESSAGE_INFO_HUMIDITY_STATUS, MAVLINK_MESSAGE_INFO_UAVCAN_NETWORK_STATUS, MAVLINK_MESSAGE_INFO_UAVCAN_NETWORK_STATISTICS, MAVLINK_MESSAGE_INFO_WIRELESS_STATISTICS, MAVLINK_MESSAGE_INFO_MASTER_MAVLINK_NOTIFICATION, MAVLINK_MESSAGE_INFO_GET_COMMAND, MAVLINK_MESSAGE_INFO_PX4GPIO_SET_CMD, MAVLINK_MESSAGE_INFO_SLAVE_ENABLE_CMD, MAVLINK_MESSAGE_INFO_FILE_TRANSFER_PROTOCOL, MAVLINK_MESSAGE_INFO_ACK, MAVLINK_MESSAGE_INFO_GET_COMMAND_NACK}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_RPSM_FIRMWARE_MASTER_H
