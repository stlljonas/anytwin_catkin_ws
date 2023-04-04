/*!
 * @file    RPSMWirelessCommunication.cpp
 * @author  Russell Buchanan
 * @date    Dec 1, 2016
 */

#include "rpsm_wireless_communication/RPSMWirelessCommunication.hpp"

namespace rpsm_wireless_communication {

RPSMWirelessCommunication::RPSMWirelessCommunication(NodeHandlePtr nh): 
	any_node::Node(nh)
{
}

RPSMWirelessCommunication::~RPSMWirelessCommunication()
{
}

bool RPSMWirelessCommunication::init() {

	// Initialize Xbee
	fd = xbee_init(port,&tty);

	sendBuffer = (uint8_t *)calloc(sendBytes,sizeof(uint8_t));

	recBytes = 96;
	sendBytes = 5;

	buttonState[0] = 0x00;
	buttonState[1] = 0x00;

	if (fd>-1){

		ROS_INFO("Xbee Initialized Successfully");

		initializePublishers();

	}else{
		ROS_ERROR("Error Setting up Xbee");
		return false;
	}
	
	return addWorker(ros::this_node::getName() + "::updateWorker", param<double>("time_step", 1.0), &RPSMWirelessCommunication::update, this, 90);
}

void RPSMWirelessCommunication::initializePublishers() {

	wirelessCommunicaitonStatusPublisher_ = 
	advertise<rpsm_wireless_communication::WirelessCommunicationStatus>
	("rpsm_wireless_communicaiton_status","/rpsm_wireless_communicaiton_status", 10);

	wirelessCommunicaitonPublisher_ = 
	advertise<rpsm_wireless_communication::RPSMStatus>("rpsm_status","/rpsm_status", 10);

	wirelessCommunicationService_ =	
	advertiseService("rpsm_service","/rpsm_service", &RPSMWirelessCommunication::serviceCallback,this);

    ROS_INFO("Publisher Initialized");
}

bool RPSMWirelessCommunication::serviceCallback
		(rpsm_wireless_communication::RPSMService::Request  &req,
         rpsm_wireless_communication::RPSMService::Response &res) {

	const static uint8_t starting_mark = 0xAA;

	//sendBuffer = (uint8_t *)calloc(sendBytes,sizeof(uint8_t));

	uint8_t firstByte = 0;
	uint8_t secondByte = 0;

	// First Byte
	if(req.battery){firstByte = (firstByte | 0x01);}
	firstByte = firstByte << 1;

	if(req.rail24V){firstByte = (firstByte | 0x01);}
	firstByte = firstByte << 1;

	if(req.rail15V){firstByte = (firstByte | 0x01);}
	firstByte = firstByte << 1;

	if(req.rail12V){firstByte = (firstByte | 0x01);}
	firstByte = firstByte << 1;

	if(req.rail5V){firstByte = (firstByte | 0x01);}
	firstByte = firstByte << 1;

	if(req.leg0){firstByte = (firstByte | 0x01);}
	firstByte = firstByte << 1;

	if(req.leg1){firstByte = (firstByte | 0x01);}
	firstByte = firstByte << 1;

	if(req.leg2){firstByte = (firstByte | 0x01);}

	// Second Byte
	if(req.leg3){secondByte = (secondByte | 0x01);}
	secondByte = secondByte << 1;

	if(req.aux0){secondByte = (secondByte | 0x01);}
	secondByte = secondByte << 1;

	if(req.aux1){secondByte = (secondByte | 0x01);}

	sendBuffer[0] = starting_mark;
	sendBuffer[1] = starting_mark;
	sendBuffer[2] = starting_mark;

	sendBuffer[3] = firstByte;
	sendBuffer[4] = secondByte;

	return true;

}

void RPSMWirelessCommunication::publish(uint8_t *message)  {

	rpsm_wireless_communication::RPSMStatus data;

	data.header.stamp = ros::Time::now();

	data.systemStatus = message[4];

	data.lomoPCStatus = message[5];
	data.naviPCStatus = message[6];
	data.operatorPCStatus = message[7];
	data.aux0PCStatus = message[8];
	data.aux1PCStatus = message[9];

	data.battery.status = message[10];
	data.battery.name = "Battery";
	data.battery.remainingPwr = message[11];
	data.battery.remainingTime = convertToUint16(message[12],message[13]);
	data.battery.voltage = convertToFloat32(message[14],message[15]);
	data.battery.current = convertToFloat32(message[16],message[17]);	
	data.battery.power = convertToUint16(message[18],message[19]);

	data.powerRail.status = message[20];
	data.powerRail.name = "24V";
	data.powerRail.voltage = convertToFloat32(message[21],message[22]);
	data.powerRail.current = convertToFloat32(message[23],message[24]);
	data.powerRail.power = convertToUint16(message[25],message[26]);
	data.powerRail.temperature = message[27];

	data.actuator.status = message[52];
	data.actuator.name = "L0";
	data.actuator.voltage = convertToFloat32(message[52],message[54]);
	data.actuator.current = convertToFloat32(message[55],message[56]);
	data.actuator.power = convertToUint16(message[57],message[58]);

	buttonState[0] = message[94];
	buttonState[1] = message[95];

	wirelessCommunicaitonPublisher_.publish(data);


	//Wireless communication Information
	rpsm_wireless_communication::WirelessCommunicationStatus status;

	status.header.stamp = ros::Time::now();
	status.sentPacketCount = oldCount;
	status.validPacketCount = validMsgCount;
	status.corruptPacketeCount = corruptMsgCount;
	wirelessCommunicaitonStatusPublisher_.publish(status);

}

bool RPSMWirelessCommunication::receiveWirelessData(){

	uint8_t *recBuffer;
	recBuffer = (uint8_t *)malloc(recBytes);

	int n = xbee_read(fd, recBuffer, recBytes);

	if(n>0){// Valid message
		publish(recBuffer);
		validMsgCount++;
		return true;
	}else if (n<0){// Corrupt message
		corruptMsgCount++;
	}

	//No message
	return false;

}

bool RPSMWirelessCommunication::sendWirelessData(){

	int n = xbee_write(fd, sendBuffer, sendBytes);

	return true;
}

bool RPSMWirelessCommunication::update(const any_worker::WorkerEvent& event) {

	// Always check for new wireless data
	receiveWirelessData();

	if ((buttonState[0] != sendBuffer[3]) || (buttonState[1] != sendBuffer[4]) && (validMsgCount > oldCount)){
		sendWirelessData();
		// sendWirelessData();
		// sendWirelessData();
		oldCount = validMsgCount;
		//ROS_INFO("buttonState: %x, %x sendBuffer: %x, %x\n",buttonState[0], buttonState[1], sendBuffer[3],sendBuffer[4] );
	}

	return true;
}

void RPSMWirelessCommunication::cleanup() {
	xbee_close(fd);
}

} /* namespace m545_wireless_communication */