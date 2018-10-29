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
  backend driver class for ActuatorStatus
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_ActuatorStatus.h"
#include "AP_ActuatorStatus_Backend.h"

extern const AP_HAL::HAL &hal;

AP_ActuatorStatus_Backend::AP_ActuatorStatus_Backend(AP_ActuatorStatus &_frontend, uint8_t _instance) :
    frontend(_frontend),
    instance(_instance)
{
}

AP_ActuatorStatus_Backend::~AP_ActuatorStatus_Backend(void)
{
}
