#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend_Serial.h"

class AP_RangeFinder_LightWareSerial : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);

    char linebuf[10];
    uint8_t linebuf_len = 0;
    uint32_t last_init_ms;
};
