#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_ActuatorStatus_UAVCAN.h"

#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>

#include <uavcan/equipment/actuator/Status.hpp>

extern const AP_HAL::HAL& hal;

#define debug_actuator_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { printf(fmt, ##args); }} while (0)


// singleton instance
//AP_ActuatorStatus *AP_ActuatorStatus::_instance;

// UAVCAN Frontend Registry Binder
UC_REGISTRY_BINDER(ActuatorCb, uavcan::equipment::actuator::Status);

AP_ActuatorStatus_UAVCAN::DetectedModules AP_ActuatorStatus_UAVCAN::_detected_modules[] = {0};
HAL_Semaphore AP_ActuatorStatus_UAVCAN::_sem_registry;

// constructor
AP_ActuatorStatus_UAVCAN::AP_ActuatorStatus_UAVCAN(AP_ActuatorStatus &_frontend, uint8_t _instance) :
	AP_ActuatorStatus_Backend(_frontend, _instance)
{}

void AP_ActuatorStatus_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::actuator::Status, ActuatorCb> *actuator_listener;
    actuator_listener = new uavcan::Subscriber<uavcan::equipment::actuator::Status, ActuatorCb>(*node);

    const int actuator_listener_res = actuator_listener->start(ActuatorCb(ap_uavcan, &handle_actuator));
    if (actuator_listener_res < 0) {
        AP_HAL::panic("UAVCAN Actuator_listener_res subscriber start problem\n");
    }
}

bool AP_ActuatorStatus_UAVCAN::take_registry()
{
    return _sem_registry.take(HAL_SEMAPHORE_BLOCK_FOREVER);
}

void AP_ActuatorStatus_UAVCAN::give_registry()
{
    _sem_registry.give();
}

AP_ActuatorStatus_Backend* AP_ActuatorStatus_UAVCAN::probe(AP_ActuatorStatus &_frontend, uint8_t _instance)
{
    if (!take_registry()) {
        return nullptr;
    }

    AP_ActuatorStatus_UAVCAN* backend = nullptr;

    for (uint8_t i = 0; i < ACTUATORSTATUS_MAX_SENSORS; i++) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_uavcan != nullptr) {
            backend = new AP_ActuatorStatus_UAVCAN(_frontend, _instance);
            if (backend == nullptr) {
            	debug_actuator_uavcan(2,
                                      _detected_modules[i].ap_uavcan->get_driver_index(),
                                      "Failed register UAVCAN actuator status Node %d on Bus %d\n",
                                      _detected_modules[i].node_id,
                                      _detected_modules[i].ap_uavcan->get_driver_index());
            } else {
                _detected_modules[i].driver = backend;
                debug_actuator_uavcan(2,
                                      _detected_modules[i].ap_uavcan->get_driver_index(),
                                      "Registered UAVCAN actuator status Node %d on Bus %d\n",
                                      _detected_modules[i].node_id,
                                      _detected_modules[i].ap_uavcan->get_driver_index());
            }
            break;
        }
    }

    give_registry();

    return backend;
}

AP_ActuatorStatus_UAVCAN* AP_ActuatorStatus_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }

    for (uint8_t i = 0; i < ACTUATORSTATUS_MAX_SENSORS; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_uavcan == ap_uavcan &&
            _detected_modules[i].node_id == node_id ) {
            return _detected_modules[i].driver;
        }
    }

    bool detected = false;
    for (uint8_t i = 0; i < ACTUATORSTATUS_MAX_SENSORS; i++) {
        if (_detected_modules[i].ap_uavcan == ap_uavcan && _detected_modules[i].node_id == node_id) {
            // detected
            detected = true;
            break;
        }
    }

    if (!detected) {
        for (uint8_t i = 0; i < ACTUATORSTATUS_MAX_SENSORS; i++) {
            if (_detected_modules[i].ap_uavcan == nullptr) {
                _detected_modules[i].ap_uavcan = ap_uavcan;
                _detected_modules[i].node_id = node_id;
                break;
            }
        }
    }

    return nullptr;
}

void AP_ActuatorStatus_UAVCAN::handle_actuator(AP_UAVCAN* ap_uavcan, uint8_t node_id, const ActuatorCb &cb)
{
    if (take_registry()) {
    	AP_ActuatorStatus_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id);

        if (driver != nullptr) {
            WITH_SEMAPHORE(driver->_sem_actuator);
            driver->_id = cb.msg->actuator_id;
            driver->_position = cb.msg->position;
            driver->_last_update_ms = AP_HAL::millis();
        }

        give_registry();
    }
}

bool AP_ActuatorStatus_UAVCAN::init()
{
    // always returns true
    return true;
}

bool AP_ActuatorStatus_UAVCAN::get_position(float &position, uint8_t &ch)
{
    WITH_SEMAPHORE(_sem_actuator);

    if ((AP_HAL::millis() - _last_update_ms) > 1000) {
        return false;
    }

    position = _position;
    ch = _id;

    return true;
}

#endif
