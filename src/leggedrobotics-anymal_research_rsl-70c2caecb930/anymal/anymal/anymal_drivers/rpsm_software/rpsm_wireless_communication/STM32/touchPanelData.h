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

#ifndef TOUCH_PANEL_DATA_H
#define TOUCH_PANEL_DATA_H

#include "batteryData.h"
#include "powerRailData.h"
#include "actuatorData.h"

typedef struct touchPanelData_ {

    // status byte for the whole system
    uint8_t systemStatus_;

    // onboard PC status
    uint8_t lomoPCStatus_;
    uint8_t naviPCStatus_;
    uint8_t operatorPCStatus_;
    uint8_t aux0PCStatus_;
    uint8_t aux1PCStatus_;

    // battery data
    batteryData  battery_;

    // power rails of different voltages
    powerRailData  rail_24V_;
    powerRailData  rail_15V_;
    powerRailData  rail_12V_;
    powerRailData  rail_5V_;

    // actuator boards (4 legs + 2 spare)
    actuatorData AB_L0_;
    actuatorData AB_L1_;
    actuatorData AB_L2_;
    actuatorData AB_L3_;
    actuatorData AB_Aux0_;
    actuatorData AB_Aux1_;


} touchPanelData;


#endif