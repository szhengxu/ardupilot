#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

class AP_ActuatorStatus_Backend;

#define ACTUATORSTATUS_MAX_SENSORS 4

class AP_ActuatorStatus
{
public:
    friend class AP_ActuatorStatus_Backend;
    
    // constructor
    AP_ActuatorStatus();

    void init(void);

    // get position if available
    bool get_position(uint8_t i, float &position, uint8_t &ch);
    
private:
    static AP_ActuatorStatus *_singleton;

    AP_ActuatorStatus_Backend *sensor[ACTUATORSTATUS_MAX_SENSORS];
};
