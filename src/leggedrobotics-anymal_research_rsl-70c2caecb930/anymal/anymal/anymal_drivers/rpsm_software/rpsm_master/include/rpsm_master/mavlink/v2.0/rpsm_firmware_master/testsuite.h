/** @file
 *    @brief MAVLink comm protocol testsuite generated from rpsm_firmware_master.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef RPSM_FIRMWARE_MASTER_TESTSUITE_H
#define RPSM_FIRMWARE_MASTER_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_rpsm_firmware_master(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_rpsm_firmware_master(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_heartbeat(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HEARTBEAT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_heartbeat_t packet_in = {
        17235,17339,17443,17547,29,96,163,230,2
    };
    mavlink_heartbeat_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.subsystem_status = packet_in.subsystem_status;
        packet1.errors = packet_in.errors;
        packet1.bms_voltage_mv = packet_in.bms_voltage_mv;
        packet1.bms_current_ma = packet_in.bms_current_ma;
        packet1.type = packet_in.type;
        packet1.master_status = packet_in.master_status;
        packet1.bms_working = packet_in.bms_working;
        packet1.bms_stateofcharge = packet_in.bms_stateofcharge;
        packet1.mavlink_version = packet_in.mavlink_version;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_heartbeat_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg , packet1.type , packet1.master_status , packet1.subsystem_status , packet1.errors , packet1.bms_working , packet1.bms_voltage_mv , packet1.bms_current_ma , packet1.bms_stateofcharge );
    mavlink_msg_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_heartbeat_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.master_status , packet1.subsystem_status , packet1.errors , packet1.bms_working , packet1.bms_voltage_mv , packet1.bms_current_ma , packet1.bms_stateofcharge );
    mavlink_msg_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_heartbeat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_heartbeat_send(MAVLINK_COMM_1 , packet1.type , packet1.master_status , packet1.subsystem_status , packet1.errors , packet1.bms_working , packet1.bms_voltage_mv , packet1.bms_current_ma , packet1.bms_stateofcharge );
    mavlink_msg_heartbeat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_master_circuit_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_master_circuit_status_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,963498504,963498712,963498920,963499128,113,180,247
    };
    mavlink_master_circuit_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.voltage_mv_5V = packet_in.voltage_mv_5V;
        packet1.voltage_mv_12V = packet_in.voltage_mv_12V;
        packet1.voltage_mv_15V = packet_in.voltage_mv_15V;
        packet1.current_ma_5V = packet_in.current_ma_5V;
        packet1.current_ma_12V = packet_in.current_ma_12V;
        packet1.current_ma_15V = packet_in.current_ma_15V;
        packet1.temperature_5V = packet_in.temperature_5V;
        packet1.temperature_12V = packet_in.temperature_12V;
        packet1.temperature_15V = packet_in.temperature_15V;
        packet1.operational_5V = packet_in.operational_5V;
        packet1.operational_12V = packet_in.operational_12V;
        packet1.operational_15V = packet_in.operational_15V;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_master_circuit_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_master_circuit_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_master_circuit_status_pack(system_id, component_id, &msg , packet1.voltage_mv_5V , packet1.voltage_mv_12V , packet1.voltage_mv_15V , packet1.current_ma_5V , packet1.current_ma_12V , packet1.current_ma_15V , packet1.temperature_5V , packet1.temperature_12V , packet1.temperature_15V , packet1.operational_5V , packet1.operational_12V , packet1.operational_15V );
    mavlink_msg_master_circuit_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_master_circuit_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.voltage_mv_5V , packet1.voltage_mv_12V , packet1.voltage_mv_15V , packet1.current_ma_5V , packet1.current_ma_12V , packet1.current_ma_15V , packet1.temperature_5V , packet1.temperature_12V , packet1.temperature_15V , packet1.operational_5V , packet1.operational_12V , packet1.operational_15V );
    mavlink_msg_master_circuit_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_master_circuit_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_master_circuit_status_send(MAVLINK_COMM_1 , packet1.voltage_mv_5V , packet1.voltage_mv_12V , packet1.voltage_mv_15V , packet1.current_ma_5V , packet1.current_ma_12V , packet1.current_ma_15V , packet1.temperature_5V , packet1.temperature_12V , packet1.temperature_15V , packet1.operational_5V , packet1.operational_12V , packet1.operational_15V );
    mavlink_msg_master_circuit_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_slave_circuit_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_slave_circuit_status_t packet_in = {
        { 963497464, 963497465, 963497466, 963497467 },{ 963498296, 963498297, 963498298, 963498299 },{ 963499128, 963499129, 963499130, 963499131 },{ 149, 150, 151, 152 },{ 161, 162, 163, 164 }
    };
    mavlink_slave_circuit_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.voltage_mv_48V, packet_in.voltage_mv_48V, sizeof(uint32_t)*4);
        mav_array_memcpy(packet1.current_ma_48V, packet_in.current_ma_48V, sizeof(int32_t)*4);
        mav_array_memcpy(packet1.temperature_48V, packet_in.temperature_48V, sizeof(uint32_t)*4);
        mav_array_memcpy(packet1.slave_id, packet_in.slave_id, sizeof(uint8_t)*4);
        mav_array_memcpy(packet1.estop, packet_in.estop, sizeof(uint8_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slave_circuit_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_slave_circuit_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slave_circuit_status_pack(system_id, component_id, &msg , packet1.slave_id , packet1.voltage_mv_48V , packet1.current_ma_48V , packet1.temperature_48V , packet1.estop );
    mavlink_msg_slave_circuit_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slave_circuit_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.slave_id , packet1.voltage_mv_48V , packet1.current_ma_48V , packet1.temperature_48V , packet1.estop );
    mavlink_msg_slave_circuit_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_slave_circuit_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slave_circuit_status_send(MAVLINK_COMM_1 , packet1.slave_id , packet1.voltage_mv_48V , packet1.current_ma_48V , packet1.temperature_48V , packet1.estop );
    mavlink_msg_slave_circuit_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_bms_circuit_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_bms_circuit_status_t packet_in = {
        963497464,963497672,17651,17755,17859,17963,18067,18171,18275,18379,18483,18587,18691,18795,18899,19003,19107,19211,19315,3
    };
    mavlink_bms_circuit_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.safetystatus = packet_in.safetystatus;
        packet1.operationstatus = packet_in.operationstatus;
        packet1.serialnumber = packet_in.serialnumber;
        packet1.temperature = packet_in.temperature;
        packet1.voltage = packet_in.voltage;
        packet1.current = packet_in.current;
        packet1.batterystatus = packet_in.batterystatus;
        packet1.cellvoltage1 = packet_in.cellvoltage1;
        packet1.cellvoltage2 = packet_in.cellvoltage2;
        packet1.cellvoltage3 = packet_in.cellvoltage3;
        packet1.cellvoltage4 = packet_in.cellvoltage4;
        packet1.cellvoltage5 = packet_in.cellvoltage5;
        packet1.cellvoltage6 = packet_in.cellvoltage6;
        packet1.cellvoltage7 = packet_in.cellvoltage7;
        packet1.cellvoltage8 = packet_in.cellvoltage8;
        packet1.cellvoltage9 = packet_in.cellvoltage9;
        packet1.cellvoltage10 = packet_in.cellvoltage10;
        packet1.cellvoltage11 = packet_in.cellvoltage11;
        packet1.cellvoltage12 = packet_in.cellvoltage12;
        packet1.stateofcharge = packet_in.stateofcharge;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_bms_circuit_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_bms_circuit_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_bms_circuit_status_pack(system_id, component_id, &msg , packet1.serialnumber , packet1.temperature , packet1.voltage , packet1.current , packet1.stateofcharge , packet1.batterystatus , packet1.safetystatus , packet1.operationstatus , packet1.cellvoltage1 , packet1.cellvoltage2 , packet1.cellvoltage3 , packet1.cellvoltage4 , packet1.cellvoltage5 , packet1.cellvoltage6 , packet1.cellvoltage7 , packet1.cellvoltage8 , packet1.cellvoltage9 , packet1.cellvoltage10 , packet1.cellvoltage11 , packet1.cellvoltage12 );
    mavlink_msg_bms_circuit_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_bms_circuit_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.serialnumber , packet1.temperature , packet1.voltage , packet1.current , packet1.stateofcharge , packet1.batterystatus , packet1.safetystatus , packet1.operationstatus , packet1.cellvoltage1 , packet1.cellvoltage2 , packet1.cellvoltage3 , packet1.cellvoltage4 , packet1.cellvoltage5 , packet1.cellvoltage6 , packet1.cellvoltage7 , packet1.cellvoltage8 , packet1.cellvoltage9 , packet1.cellvoltage10 , packet1.cellvoltage11 , packet1.cellvoltage12 );
    mavlink_msg_bms_circuit_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_bms_circuit_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_bms_circuit_status_send(MAVLINK_COMM_1 , packet1.serialnumber , packet1.temperature , packet1.voltage , packet1.current , packet1.stateofcharge , packet1.batterystatus , packet1.safetystatus , packet1.operationstatus , packet1.cellvoltage1 , packet1.cellvoltage2 , packet1.cellvoltage3 , packet1.cellvoltage4 , packet1.cellvoltage5 , packet1.cellvoltage6 , packet1.cellvoltage7 , packet1.cellvoltage8 , packet1.cellvoltage9 , packet1.cellvoltage10 , packet1.cellvoltage11 , packet1.cellvoltage12 );
    mavlink_msg_bms_circuit_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_humidity_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HUMIDITY_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_humidity_status_t packet_in = {
        17.0,45.0,29
    };
    mavlink_humidity_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ambient_temperature = packet_in.ambient_temperature;
        packet1.ambient_humidity = packet_in.ambient_humidity;
        packet1.humidity_working = packet_in.humidity_working;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HUMIDITY_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HUMIDITY_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_humidity_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_humidity_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_humidity_status_pack(system_id, component_id, &msg , packet1.ambient_temperature , packet1.ambient_humidity , packet1.humidity_working );
    mavlink_msg_humidity_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_humidity_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ambient_temperature , packet1.ambient_humidity , packet1.humidity_working );
    mavlink_msg_humidity_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_humidity_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_humidity_status_send(MAVLINK_COMM_1 , packet1.ambient_temperature , packet1.ambient_humidity , packet1.humidity_working );
    mavlink_msg_humidity_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_uavcan_network_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_uavcan_network_status_t packet_in = {
        963497464,{ 963497672, 963497673, 963497674, 963497675, 963497676, 963497677, 963497678, 963497679, 963497680, 963497681, 963497682, 963497683, 963497684, 963497685, 963497686, 963497687 },209,20,87,154,221,{ 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47 },{ 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95 },{ 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143 },{ 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191 }
    };
    mavlink_uavcan_network_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.master_up_time_s = packet_in.master_up_time_s;
        packet1.nr_of_slaves = packet_in.nr_of_slaves;
        packet1.master_id = packet_in.master_id;
        packet1.master_mode = packet_in.master_mode;
        packet1.master_health = packet_in.master_health;
        packet1.master_errorflags = packet_in.master_errorflags;
        
        mav_array_memcpy(packet1.slave_uptime_s, packet_in.slave_uptime_s, sizeof(uint32_t)*16);
        mav_array_memcpy(packet1.slave_ids, packet_in.slave_ids, sizeof(uint8_t)*16);
        mav_array_memcpy(packet1.slave_mode, packet_in.slave_mode, sizeof(uint8_t)*16);
        mav_array_memcpy(packet1.slave_health, packet_in.slave_health, sizeof(uint8_t)*16);
        mav_array_memcpy(packet1.slave_errorflags, packet_in.slave_errorflags, sizeof(uint8_t)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UAVCAN_NETWORK_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_network_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_uavcan_network_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_network_status_pack(system_id, component_id, &msg , packet1.nr_of_slaves , packet1.master_id , packet1.master_mode , packet1.master_health , packet1.master_errorflags , packet1.master_up_time_s , packet1.slave_ids , packet1.slave_mode , packet1.slave_health , packet1.slave_errorflags , packet1.slave_uptime_s );
    mavlink_msg_uavcan_network_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_network_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.nr_of_slaves , packet1.master_id , packet1.master_mode , packet1.master_health , packet1.master_errorflags , packet1.master_up_time_s , packet1.slave_ids , packet1.slave_mode , packet1.slave_health , packet1.slave_errorflags , packet1.slave_uptime_s );
    mavlink_msg_uavcan_network_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_uavcan_network_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_network_status_send(MAVLINK_COMM_1 , packet1.nr_of_slaves , packet1.master_id , packet1.master_mode , packet1.master_health , packet1.master_errorflags , packet1.master_up_time_s , packet1.slave_ids , packet1.slave_mode , packet1.slave_health , packet1.slave_errorflags , packet1.slave_uptime_s );
    mavlink_msg_uavcan_network_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_uavcan_network_statistics(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_uavcan_network_statistics_t packet_in = {
        963497464,963497672,963497880
    };
    mavlink_uavcan_network_statistics_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.rx_msgs = packet_in.rx_msgs;
        packet1.tx_msgs = packet_in.tx_msgs;
        packet1.errors = packet_in.errors;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UAVCAN_NETWORK_STATISTICS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_network_statistics_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_uavcan_network_statistics_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_network_statistics_pack(system_id, component_id, &msg , packet1.rx_msgs , packet1.tx_msgs , packet1.errors );
    mavlink_msg_uavcan_network_statistics_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_network_statistics_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rx_msgs , packet1.tx_msgs , packet1.errors );
    mavlink_msg_uavcan_network_statistics_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_uavcan_network_statistics_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavcan_network_statistics_send(MAVLINK_COMM_1 , packet1.rx_msgs , packet1.tx_msgs , packet1.errors );
    mavlink_msg_uavcan_network_statistics_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_wireless_statistics(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WIRELESS_STATISTICS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_wireless_statistics_t packet_in = {
        963497464,963497672,17651,17755
    };
    mavlink_wireless_statistics_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.rx_bytes = packet_in.rx_bytes;
        packet1.tx_bytes = packet_in.tx_bytes;
        packet1.rx_msgs = packet_in.rx_msgs;
        packet1.rx_errors = packet_in.rx_errors;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WIRELESS_STATISTICS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WIRELESS_STATISTICS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wireless_statistics_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_wireless_statistics_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wireless_statistics_pack(system_id, component_id, &msg , packet1.rx_bytes , packet1.tx_bytes , packet1.rx_msgs , packet1.rx_errors );
    mavlink_msg_wireless_statistics_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wireless_statistics_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rx_bytes , packet1.tx_bytes , packet1.rx_msgs , packet1.rx_errors );
    mavlink_msg_wireless_statistics_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_wireless_statistics_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wireless_statistics_send(MAVLINK_COMM_1 , packet1.rx_bytes , packet1.tx_bytes , packet1.rx_msgs , packet1.rx_errors );
    mavlink_msg_wireless_statistics_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_master_mavlink_notification(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_master_mavlink_notification_t packet_in = {
        963497464,17,"FGHIJKLMN","PQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKL"
    };
    mavlink_master_mavlink_notification_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.code = packet_in.code;
        packet1.level = packet_in.level;
        
        mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*10);
        mav_array_memcpy(packet1.description, packet_in.description, sizeof(char)*50);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_master_mavlink_notification_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_master_mavlink_notification_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_master_mavlink_notification_pack(system_id, component_id, &msg , packet1.code , packet1.level , packet1.name , packet1.description );
    mavlink_msg_master_mavlink_notification_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_master_mavlink_notification_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.code , packet1.level , packet1.name , packet1.description );
    mavlink_msg_master_mavlink_notification_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_master_mavlink_notification_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_master_mavlink_notification_send(MAVLINK_COMM_1 , packet1.code , packet1.level , packet1.name , packet1.description );
    mavlink_msg_master_mavlink_notification_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_get_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GET_COMMAND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_get_command_t packet_in = {
        17235,139
    };
    mavlink_get_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.get_type = packet_in.get_type;
        packet1.slave_id = packet_in.slave_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GET_COMMAND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GET_COMMAND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_command_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_get_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_command_pack(system_id, component_id, &msg , packet1.slave_id , packet1.get_type );
    mavlink_msg_get_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.slave_id , packet1.get_type );
    mavlink_msg_get_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_get_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_command_send(MAVLINK_COMM_1 , packet1.slave_id , packet1.get_type );
    mavlink_msg_get_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_px4gpio_set_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PX4GPIO_SET_CMD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_px4gpio_set_cmd_t packet_in = {
        5,72,139,206,17,84,151
    };
    mavlink_px4gpio_set_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.main_enable = packet_in.main_enable;
        packet1.enable_can_pwr = packet_in.enable_can_pwr;
        packet1.enable_exdev = packet_in.enable_exdev;
        packet1.enable_5v = packet_in.enable_5v;
        packet1.enable_12v  = packet_in.enable_12v ;
        packet1.enable_15v = packet_in.enable_15v;
        packet1.mainled_enable = packet_in.mainled_enable;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PX4GPIO_SET_CMD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PX4GPIO_SET_CMD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_px4gpio_set_cmd_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_px4gpio_set_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_px4gpio_set_cmd_pack(system_id, component_id, &msg , packet1.main_enable , packet1.enable_can_pwr , packet1.enable_exdev , packet1.enable_5v , packet1.enable_12v  , packet1.enable_15v , packet1.mainled_enable );
    mavlink_msg_px4gpio_set_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_px4gpio_set_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.main_enable , packet1.enable_can_pwr , packet1.enable_exdev , packet1.enable_5v , packet1.enable_12v  , packet1.enable_15v , packet1.mainled_enable );
    mavlink_msg_px4gpio_set_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_px4gpio_set_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_px4gpio_set_cmd_send(MAVLINK_COMM_1 , packet1.main_enable , packet1.enable_can_pwr , packet1.enable_exdev , packet1.enable_5v , packet1.enable_12v  , packet1.enable_15v , packet1.mainled_enable );
    mavlink_msg_px4gpio_set_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_slave_enable_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SLAVE_ENABLE_CMD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_slave_enable_cmd_t packet_in = {
        5,72
    };
    mavlink_slave_enable_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.slave_id = packet_in.slave_id;
        packet1.set_cmd = packet_in.set_cmd;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SLAVE_ENABLE_CMD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slave_enable_cmd_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_slave_enable_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slave_enable_cmd_pack(system_id, component_id, &msg , packet1.slave_id , packet1.set_cmd );
    mavlink_msg_slave_enable_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slave_enable_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.slave_id , packet1.set_cmd );
    mavlink_msg_slave_enable_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_slave_enable_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slave_enable_cmd_send(MAVLINK_COMM_1 , packet1.slave_id , packet1.set_cmd );
    mavlink_msg_slave_enable_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_file_transfer_protocol(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_file_transfer_protocol_t packet_in = {
        5,72,139,{ 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200 }
    };
    mavlink_file_transfer_protocol_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_network = packet_in.target_network;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.payload, packet_in.payload, sizeof(uint8_t)*251);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_transfer_protocol_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_file_transfer_protocol_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_transfer_protocol_pack(system_id, component_id, &msg , packet1.target_network , packet1.target_system , packet1.target_component , packet1.payload );
    mavlink_msg_file_transfer_protocol_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_transfer_protocol_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_network , packet1.target_system , packet1.target_component , packet1.payload );
    mavlink_msg_file_transfer_protocol_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_file_transfer_protocol_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_file_transfer_protocol_send(MAVLINK_COMM_1 , packet1.target_network , packet1.target_system , packet1.target_component , packet1.payload );
    mavlink_msg_file_transfer_protocol_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ack_t packet_in = {
        5
    };
    mavlink_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.reason = packet_in.reason;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ack_pack(system_id, component_id, &msg , packet1.reason );
    mavlink_msg_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.reason );
    mavlink_msg_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ack_send(MAVLINK_COMM_1 , packet1.reason );
    mavlink_msg_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_get_command_nack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GET_COMMAND_NACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_get_command_nack_t packet_in = {
        5
    };
    mavlink_get_command_nack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.error_number = packet_in.error_number;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GET_COMMAND_NACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GET_COMMAND_NACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_command_nack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_get_command_nack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_command_nack_pack(system_id, component_id, &msg , packet1.error_number );
    mavlink_msg_get_command_nack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_command_nack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.error_number );
    mavlink_msg_get_command_nack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_get_command_nack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_get_command_nack_send(MAVLINK_COMM_1 , packet1.error_number );
    mavlink_msg_get_command_nack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rpsm_firmware_master(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_heartbeat(system_id, component_id, last_msg);
    mavlink_test_master_circuit_status(system_id, component_id, last_msg);
    mavlink_test_slave_circuit_status(system_id, component_id, last_msg);
    mavlink_test_bms_circuit_status(system_id, component_id, last_msg);
    mavlink_test_humidity_status(system_id, component_id, last_msg);
    mavlink_test_uavcan_network_status(system_id, component_id, last_msg);
    mavlink_test_uavcan_network_statistics(system_id, component_id, last_msg);
    mavlink_test_wireless_statistics(system_id, component_id, last_msg);
    mavlink_test_master_mavlink_notification(system_id, component_id, last_msg);
    mavlink_test_get_command(system_id, component_id, last_msg);
    mavlink_test_px4gpio_set_cmd(system_id, component_id, last_msg);
    mavlink_test_slave_enable_cmd(system_id, component_id, last_msg);
    mavlink_test_file_transfer_protocol(system_id, component_id, last_msg);
    mavlink_test_ack(system_id, component_id, last_msg);
    mavlink_test_get_command_nack(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // RPSM_FIRMWARE_MASTER_TESTSUITE_H
