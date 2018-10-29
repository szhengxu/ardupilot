#pragma once

#include "AP_ActuatorStatus_Backend.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

class ActuatorCb;

class AP_ActuatorStatus_UAVCAN : public AP_ActuatorStatus_Backend {
public:
	AP_ActuatorStatus_UAVCAN(AP_ActuatorStatus &_frontend, uint8_t _instance);

	bool init(void) override;

	bool get_position(float &position, uint8_t &ch) override;

	static void subscribe_msgs(AP_UAVCAN* ap_uavcan);

	static AP_ActuatorStatus_Backend* probe(AP_ActuatorStatus &_fronted, uint8_t _instance);

private:

	static bool take_registry();

	static void give_registry();

	static void handle_actuator(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ActuatorCb &cb);

	static AP_ActuatorStatus_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id);

	uint8_t _id;
	float _position;
	uint32_t _last_update_ms;

	HAL_Semaphore _sem_actuator;

    // Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        AP_ActuatorStatus_UAVCAN *driver;
    } _detected_modules[ACTUATORSTATUS_MAX_SENSORS];

	static HAL_Semaphore _sem_registry;
};
