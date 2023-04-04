# Battery Manager State

#battery monitor sensor serial number report in Hex
uint16 serialnumber

#battery monitor sensor temperature report in [0.1 K]
uint16 temperature

#battery monitor sensor voltage report in [mV]
uint16 voltage

#battery monitor sensor current report in [mA]
int16  current

#battery monitor sensor state of charge in [%]
uint8  stateofcharge

#battery monitor sensor battery status report in Hex
uint16 batterystatus

#battery monitor sensor safety status report in Hex
uint32 safetystatus

#battery monitor sensor operation status report in Hex
uint32 operationstatus

#battery monitor sensor cell voltages report in [mV]
uint16[] cellvoltage
