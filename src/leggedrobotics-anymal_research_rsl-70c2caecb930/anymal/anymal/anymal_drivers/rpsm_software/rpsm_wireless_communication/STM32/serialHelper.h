/******************************************************************************
 * Copyright (C) 2016 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#ifndef SERIAL_HELPER_H
#define SERIAL_HELPER_H


/**
 *  @brief some useful helper functions
 */
uint8_t getInt(float l){
   return (uint8_t) l;
}

uint8_t getDecimal(float l){
    return (uint8_t)(100*(l-getInt(l)));
}


uint8_t getHigherByte(uint16_t l){
    return (uint8_t) (l >> 8);
}

uint8_t getLowerByte(uint16_t l){
    return (uint8_t) l ;
}


/**
 *  @brief try to extract package from serial data stream
 *  @param      c: next byte received over serial RX
 *         buffer: uint8_t array to temporarily store package
 *                 need AT LEAST 120-byte space!
 *  @retval 0 if no package detected,
 *          1 if package IS detected
 */
uint8_t serialParseChar(uint8_t c, uint8_t * buffer){

   /**
   * Variables for datalink frame decoding state machine
   */
   enum StateMachine{
	state_wait,
	state_start,
	state_data,
	state_crc,
        state_decode
   };

   static enum StateMachine state = state_wait;
   const static uint8_t starting_mark = 0xAA;
   static uint8_t pkgWaitCnt = 0;
   static uint8_t pkgDataCnt = 0;
   static uint8_t len = 0;
   uint8_t foundPkg = 0;

    switch(state){
        case state_wait:
            if(c == starting_mark)
                pkgWaitCnt += 1;
                    else
                pkgWaitCnt = 0;

            if(pkgWaitCnt == 3){
                pkgWaitCnt = 0;
                state = state_start;
            }
            break;
        case state_start:
            len = c;
            pkgDataCnt = 0;
            state = state_data;
            break;
        case state_data:
            buffer[pkgDataCnt++] = c;
            if(pkgDataCnt == len){
                state = state_crc;                 
            }
            break;
         case state_crc:
             // implement CRC if needed
             foundPkg = 1;
             state = state_wait;
             break;
         default:
             state = state_wait;
             break;
    }

    return foundPkg;
}


/**
 *  @brief pack touch panel data into a buffer, ready for transmissoin over serial port
 *  @param   data_: touchPanelData struct containing all the necessary data
 *         buffer_: uint8_t array to temporarily store package
 *                 need AT LEAST 120-byte space!
 *  @retval length of serial data package 
 */
uint8_t packSerialDataPackage(touchPanelData data_, uint8_t * buffer){

    const static uint8_t starting_mark = 0xAA;
    buffer[0] = starting_mark;
    buffer[1] = starting_mark;
    buffer[2] = starting_mark;

    buffer[3] = 90;

    // full system status
    buffer[4] = data_.systemStatus_;

    // onboard PCs
    buffer[5] = data_.lomoPCStatus_;
    buffer[6] = data_.naviPCStatus_;
    buffer[7] = data_.operatorPCStatus_;
    buffer[8] = data_.aux0PCStatus_;
    buffer[9] = data_.aux1PCStatus_;

    // battery 
    buffer[10] = data_.battery_.status_;
    buffer[11] = data_.battery_.remainingPwr_;
    buffer[12] = getHigherByte( data_.battery_.remainingTime_ );
    buffer[13] = getLowerByte( data_.battery_.remainingTime_ );
    buffer[14] = getInt(data_.battery_.voltage_);
    buffer[15] = getDecimal(data_.battery_.voltage_);
    buffer[16] = getInt(data_.battery_.current_);
    buffer[17] = getDecimal(data_.battery_.current_);
    buffer[18] = getHigherByte( data_.battery_.power_ );
    buffer[19] = getLowerByte( data_.battery_.power_ );

    // power rails 24V
    buffer[20] = data_.rail_24V_.status_;
    buffer[21] = getInt(data_.rail_24V_.voltage_);
    buffer[22] = getDecimal(data_.rail_24V_.voltage_);
    buffer[23] = getInt(data_.rail_24V_.current_);
    buffer[24] = getDecimal(data_.rail_24V_.current_);
    buffer[25] = getHigherByte(data_.rail_24V_.power_);
    buffer[26] = getLowerByte(data_.rail_24V_.power_);
    buffer[27] = data_.rail_24V_.temperature_;

    // power rails 15V
    buffer[28] = data_.rail_15V_.status_;
    buffer[29] = getInt(data_.rail_15V_.voltage_);
    buffer[30] = getDecimal(data_.rail_15V_.voltage_);
    buffer[31] = getInt(data_.rail_15V_.current_);
    buffer[32] = getDecimal(data_.rail_15V_.current_);
    buffer[33] = getHigherByte(data_.rail_15V_.power_);
    buffer[34] = getLowerByte(data_.rail_15V_.power_);
    buffer[35] = data_.rail_15V_.temperature_;

    // power rails 12V
    buffer[36] = data_.rail_12V_.status_;
    buffer[37] = getInt(data_.rail_12V_.voltage_);
    buffer[38] = getDecimal(data_.rail_12V_.voltage_);
    buffer[39] = getInt(data_.rail_12V_.current_);
    buffer[40] = getDecimal(data_.rail_12V_.current_);
    buffer[41] = getHigherByte(data_.rail_12V_.power_);
    buffer[42] = getLowerByte(data_.rail_12V_.power_);
    buffer[43] = data_.rail_12V_.temperature_;

    // power rails 5V
    buffer[44] = data_.rail_5V_.status_;
    buffer[45] = getInt(data_.rail_5V_.voltage_);
    buffer[46] = getDecimal(data_.rail_5V_.voltage_);
    buffer[47] = getInt(data_.rail_5V_.current_);
    buffer[48] = getDecimal(data_.rail_5V_.current_);
    buffer[49] = getHigherByte(data_.rail_5V_.power_);
    buffer[50] = getLowerByte(data_.rail_5V_.power_);
    buffer[51] = data_.rail_5V_.temperature_;

    // AB L0
    buffer[52] = data_.AB_L0_.status_;
    buffer[53] = getInt(data_.AB_L0_.voltage_);
    buffer[54] = getDecimal(data_.AB_L0_.voltage_);
    buffer[55] = getInt(data_.AB_L0_.current_);
    buffer[56] = getDecimal(data_.AB_L0_.current_);
    buffer[57] = getHigherByte(data_.AB_L0_.power_);
    buffer[58] = getLowerByte(data_.AB_L0_.power_);

    // AB L1
    buffer[59] = data_.AB_L1_.status_;
    buffer[60] = getInt(data_.AB_L1_.voltage_);
    buffer[61] = getDecimal(data_.AB_L1_.voltage_);
    buffer[62] = getInt(data_.AB_L1_.current_);
    buffer[63] = getDecimal(data_.AB_L1_.current_);
    buffer[64] = getHigherByte(data_.AB_L1_.power_);
    buffer[65] = getLowerByte(data_.AB_L1_.power_);

    // AB L2
    buffer[66] = data_.AB_L2_.status_;
    buffer[67] = getInt(data_.AB_L2_.voltage_);
    buffer[68] = getDecimal(data_.AB_L2_.voltage_);
    buffer[69] = getInt(data_.AB_L2_.current_);
    buffer[70] = getDecimal(data_.AB_L2_.current_);
    buffer[71] = getHigherByte(data_.AB_L2_.power_);
    buffer[72] = getLowerByte(data_.AB_L2_.power_);

    // AB L3
    buffer[73] = data_.AB_L3_.status_;
    buffer[74] = getInt(data_.AB_L3_.voltage_);
    buffer[75] = getDecimal(data_.AB_L3_.voltage_);
    buffer[76] = getInt(data_.AB_L3_.current_);
    buffer[77] = getDecimal(data_.AB_L3_.current_);
    buffer[78] = getHigherByte(data_.AB_L3_.power_);
    buffer[79] = getLowerByte(data_.AB_L3_.power_);

    // AB Aux0
    buffer[80] = data_.AB_Aux0_.status_;
    buffer[81] = getInt(data_.AB_Aux0_.voltage_);
    buffer[82] = getDecimal(data_.AB_Aux0_.voltage_);
    buffer[83] = getInt(data_.AB_Aux0_.current_);
    buffer[84] = getDecimal(data_.AB_Aux0_.current_);
    buffer[85] = getHigherByte(data_.AB_Aux0_.power_);
    buffer[86] = getLowerByte(data_.AB_Aux0_.power_);

    // AB Aux1
    buffer[87] = data_.AB_Aux1_.status_;
    buffer[88] = getInt(data_.AB_Aux1_.voltage_);
    buffer[89] = getDecimal(data_.AB_Aux1_.voltage_);
    buffer[90] = getInt(data_.AB_Aux1_.current_);
    buffer[91] = getDecimal(data_.AB_Aux1_.current_);
    buffer[92] = getHigherByte(data_.AB_Aux1_.power_);
    buffer[93] = getLowerByte(data_.AB_Aux1_.power_);

    // Data package has pre-defined length
    return 94;
}



/**
 *  @brief pack control request into a buffer, ready for transmissoin over serial port
 *         used to request button status from touch panel
 *  @param buffer_: uint8_t array to temporarily store package
 *                 need AT LEAST 120-byte space!
 *  @retval length of serial data package 
 */
uint8_t packSerialRequestPackage(touchPanelData data_, uint8_t * buffer){

    const static uint8_t starting_mark = 0xAA;
    buffer[0] = starting_mark;
    buffer[1] = starting_mark;
    buffer[2] = starting_mark;

    buffer[3] = 1;

    // special identifier for button status request package 
    buffer[4] = 0xEF;

    // button status request package has pre-defined length
    return 5;
}


#endif
