# Message definition for overall RPSM status
Header header

uint8 systemStatus

uint8 lomoPCStatus
uint8 naviPCStatus
uint8 operatorPCStatus
uint8 aux0PCStatus
uint8 aux1PCStatus


rpsm_wireless_communication/BatteryData battery 
rpsm_wireless_communication/Actuator actuator
rpsm_wireless_communication/PowerRail powerRail 