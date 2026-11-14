/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */


#include <stdbool.h>
#include <stdint.h>
#include "platform.h"
#include <math.h>

#if defined(USE_DRONECAN)

#include "common/utils.h"
#include "sensors/battery_sensor_dronecan.h"
#include "sensors/battery.h"

static uint16_t dronecanVbat;
static uint16_t dronecanAmperage;

uint16_t dronecanBattSensorGetVBat(void)
{
    return dronecanVbat;
}

uint16_t dronecanBattSensorGetAmperage(void)
{
    return dronecanAmperage;
}

void dronecanBatterySensorReceiveInfo(struct uavcan_equipment_power_BatteryInfo *pbatteryInfo)
{
    const uint16_t voltage = (uint16_t)roundf(pbatteryInfo->voltage * 100.0F);
    const int16_t amperage = (int16_t)roundf(pbatteryInfo->current * 100.0F);
    battery2ReceiveCanInfo(pbatteryInfo->battery_id, voltage, amperage);

    // Keep the primary CAN meter on the primary battery. A configured
    // secondary CAN battery is consumed by battery2ReceiveCanInfo above.
    const battery2Config_t *battery2 = battery2Config();
    const bool battery2UsesCan = (battery2->voltageEnabled && battery2->voltageSensor == BATTERY2_SENSOR_CAN)
        || (battery2->currentEnabled && battery2->currentSensor == BATTERY2_SENSOR_CAN);
    if (pbatteryInfo->battery_id != battery2->canBatteryId || !battery2UsesCan) {
        dronecanVbat = voltage;
        dronecanAmperage = (uint16_t)amperage;
    }
};

#endif
