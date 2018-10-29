/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *   AP_ActuatorStatus.cpp - ActuatorStatus (pitot) driver
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <utility>
#include "AP_ActuatorStatus.h"
#include "AP_ActuatorStatus_Backend.h"
#include <GCS_MAVLink/GCS.h>
#if HAL_WITH_UAVCAN
#include "AP_ActuatorStatus_UAVCAN.h"
#endif

extern const AP_HAL::HAL &hal;

AP_ActuatorStatus::AP_ActuatorStatus()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_ActuatorStatus must be singleton");
    }
    _singleton = this;
}

void AP_ActuatorStatus::init()
{
#if HAL_WITH_UAVCAN
	for (uint8_t i = 0; i < ACTUATORSTATUS_MAX_SENSORS; i++) {
		sensor[i] = AP_ActuatorStatus_UAVCAN::probe(*this, i);
		if (sensor[i] && !sensor[i]->init()) {
			gcs().send_text(MAV_SEVERITY_INFO, "ActuatorStatus %u init failed", i + 1);
			delete sensor[i];
			sensor[i] = nullptr;
		}
	}
#endif
}

// get a position reading if possible
bool AP_ActuatorStatus::get_position(uint8_t i, float &position, uint8_t &ch)
{
    if (sensor[i]) {
        return sensor[i]->get_position(position, ch);
    }
    return false;
}

// singleton instance
AP_ActuatorStatus *AP_ActuatorStatus::_singleton;
