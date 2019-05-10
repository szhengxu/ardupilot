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

#include "AP_OAPathPlanner.h"
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_Logger/AP_Logger.h>
#include "AP_OABendyRuler.h"
#include "AP_OADijkstra.h"

extern const AP_HAL::HAL &hal;

// parameter defaults
const float OA_LOOKAHEAD_DEFAULT = 50;
const float OA_MARGIN_MAX_DEFAULT = 10;

const int16_t OA_TIMEOUT_MS = 2000;             // avoidance results over 2 seconds old are ignored

const AP_Param::GroupInfo AP_OAPathPlanner::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Object Avoidance Path Planning algorithm to use
    // @Description: Enabled/disable path planning around obstacles
    // @Values: 0:Disabled,1:BendyRuler,2:Dijkstra
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1,  AP_OAPathPlanner, _type, OA_PATHPLAN_DISABLED, AP_PARAM_FLAG_ENABLE),

    // @Param: LOOKAHEAD
    // @DisplayName: Object Avoidance look ahead distance maximum
    // @Description: Object Avoidance will look this many meters ahead of vehicle
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LOOKAHEAD", 2, AP_OAPathPlanner, _lookahead, OA_LOOKAHEAD_DEFAULT),

    // @Param: MARGIN_MAX
    // @DisplayName: Object Avoidance wide margin distance
    // @Description: Object Avoidance will ignore objects more than this many meters from vehicle
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MARGIN_MAX", 3, AP_OAPathPlanner, _margin_max, OA_MARGIN_MAX_DEFAULT),

    AP_GROUPEND
};

/// Constructor
AP_OAPathPlanner::AP_OAPathPlanner()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// perform any required initialisation
void AP_OAPathPlanner::init()
{
    // run background task looking for best alternative destination
    switch (_type) {
    case OA_PATHPLAN_DISABLED:
        // do nothing
        break;
    case OA_PATHPLAN_BENDYRULER:
        if (_oabendyruler == nullptr) {
            _oabendyruler = new AP_OABendyRuler();
        }
        break;
    case OA_PATHPLAN_DIJKSTRA:
        if (_oadijkstra == nullptr) {
            _oadijkstra = new AP_OADijkstra();
        }
        break;
    }
}

// pre-arm checks that algorithms have been initialised successfully
bool AP_OAPathPlanner::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    // check if initialisation has succeeded
    switch (_type) {
    case OA_PATHPLAN_DISABLED:
        // do nothing
        break;
    case OA_PATHPLAN_BENDYRULER:
        if (_oabendyruler == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "BendyRuler OA requires reboot");
            return false;
        }
        break;
    case OA_PATHPLAN_DIJKSTRA:
        if (_oadijkstra == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "Dijkstra OA requires reboot");
            return false;
        }
        break;
    }
    return true;
}

// provides an alternative target location if path planning around obstacles is required
// returns true and updates result_loc with an intermediate location
bool AP_OAPathPlanner::mission_avoidance(const Location &current_loc,
                                         const Location &origin,
                                         const Location &destination,
                                         Location &result_origin,
                                         Location &result_destination)
{
    // exit immediately if disabled
    if (_type == OA_PATHPLAN_DISABLED) {
        return false;
    }

    WITH_SEMAPHORE(_rsem);

    if (!_thread_created) {
        // create the avoidance thread as low priority. It should soak
        // up spare CPU cycles to fill in the avoidance_result structure based
        // on requests in avoidance_request
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_OAPathPlanner::avoidance_thread, void),
                                          "avoidance",
                                          8192, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
            return false;
        }
        _thread_created = true;
    }

    const uint32_t now = AP_HAL::millis();

    // place new request for the thread to work on
    avoidance_request.current_loc = current_loc;
    avoidance_request.origin = origin;
    avoidance_request.destination = destination;
    avoidance_request.ground_speed_vec = AP::ahrs().groundspeed_vector();
    avoidance_request.request_time_ms = now;

    // return results from background thread's latest checks
    if (destination.lat == avoidance_result.destination.lat &&
        destination.lng == avoidance_result.destination.lng &&
        now - avoidance_result.result_time_ms < OA_TIMEOUT_MS) {
        // we have a result from the thread
        result_origin = avoidance_result.origin_new;
        result_destination = avoidance_result.destination_new;
        // log result
        if (avoidance_result.result_time_ms != _logged_time_ms) {
            _logged_time_ms = avoidance_result.result_time_ms;
            AP::logger().Write_OA(_type, destination, result_destination);
        }
        return avoidance_result.avoidance_needed;
    }

    // do not performance avoidance because background thread's results were
    // run against a different destination or they are simply not required
    return false;
}

// avoidance thread that continually updates the avoidance_result structure based on avoidance_request
void AP_OAPathPlanner::avoidance_thread()
{
    while (true) {

        // run at 10hz or less
        hal.scheduler->delay(100);

        Location origin_new;
        Location destination_new;
        {
            WITH_SEMAPHORE(_rsem);
            uint32_t now = AP_HAL::millis();
            if (now - avoidance_request.request_time_ms > OA_TIMEOUT_MS) {
                // this is a very old request, don't process it
                continue;
            }

            // copy request to avoid conflict with main thread
            avoidance_request2 = avoidance_request;

            // store passed in origin and destination so we can return it if object avoidance is not required
            origin_new = avoidance_request.origin;
            destination_new = avoidance_request.destination;
        }

        // run background task looking for best alternative destination
        bool res = false;
        switch (_type) {
        case OA_PATHPLAN_DISABLED:
            continue;
        case OA_PATHPLAN_BENDYRULER:
            if (_oabendyruler == nullptr) {
                continue;
            }
            _oabendyruler->set_config(_lookahead, _margin_max);
            res = _oabendyruler->update(avoidance_request2.current_loc, avoidance_request2.destination, avoidance_request2.ground_speed_vec, origin_new, destination_new);
            break;
        case OA_PATHPLAN_DIJKSTRA:
            if (_oadijkstra == nullptr) {
                continue;
            }
            _oadijkstra->set_fence_margin(_margin_max);
            res = _oadijkstra->update(avoidance_request2.current_loc, avoidance_request2.destination, origin_new, destination_new);
            break;
        }

        {
            // give the main thread the avoidance result
            WITH_SEMAPHORE(_rsem);
            avoidance_result.destination = avoidance_request2.destination;
            avoidance_result.origin_new = res ? origin_new : avoidance_result.origin_new;
            avoidance_result.destination_new = res ? destination_new : avoidance_result.destination;
            avoidance_result.result_time_ms = AP_HAL::millis();
            avoidance_result.avoidance_needed = res;
        }
    }
}

// singleton instance
AP_OAPathPlanner *AP_OAPathPlanner::_singleton;

namespace AP {

AP_OAPathPlanner *ap_oapathplanner()
{
    return AP_OAPathPlanner::get_singleton();
}

}
