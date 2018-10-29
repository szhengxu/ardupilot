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
#pragma once

/*
  backend driver class for actuator status
 */

#include <AP_Common/AP_Common.h>
#include <AP_Common/Semaphore.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_ActuatorStatus.h"

class AP_ActuatorStatus_Backend {
public:
	AP_ActuatorStatus_Backend(AP_ActuatorStatus &frontend, uint8_t instance);
    virtual ~AP_ActuatorStatus_Backend();
    
    // probe and initialise the sensor
    virtual bool init(void) = 0;

    // return the current position in degrees , if available
    virtual bool get_position(float &position, uint8_t &ch) = 0;

protected:

    // semaphore for access to shared frontend data
    HAL_Semaphore sem;
    
private:
    AP_ActuatorStatus &frontend;
    uint8_t instance;
};
