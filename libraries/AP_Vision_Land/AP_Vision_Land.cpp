#include "AP_Vision_Land.h"

AP_Vision_Land::AP_Vision_Land(){
    this->init();
}

// handle_msg - Process a LANDING_TARGET mavlink message
void AP_Vision_Land::handle_msg(mavlink_message_t* msg)
{
        // parse mavlink message
    __mavlink_landing_target_t packet;
    mavlink_msg_landing_target_decode(msg, &packet);

    this->timestamp_us = packet.time_usec;       // Unix time

    this->frame = packet.frame;              // Frame of target pos reference

    // Position of the target in _ref_frame Frame
    this->x = packet.x;
    this->y = packet.y;
    // _z = packet.z; //shouldn't need alt

    this->valid = packet.position_valid;         // Is the position valid 0=False, 1=True

    if(this->valid){
        this->timeout_begin_ms = 0;
    }

}

// void update_vision_land(){
//     // handle mavlink
//     this.handle_msg();
// }

int AP_Vision_Land::ok(){
    return this->valid;
}

float AP_Vision_Land::get_target_lat(){
    return this->x;
}

float AP_Vision_Land::get_target_lng(){
    return this->y;
}

Location AP_Vision_Land::inject_updated_waypoint(Location next_loc){
    this->have_injected = 1;
    Location new_loc = next_loc;
    new_loc.lat = this->x;
    new_loc.lng = this->y;
    return new_loc;
}

int AP_Vision_Land::waypoint_injected(){
    return this->have_injected;
}

bool AP_Vision_Land::search_timeout(){
    //if first time called, assign call time, return false
    if (this->timeout_begin_ms == 0){
        this->timeout_begin_ms = AP_HAL::millis();   // set search start time
        return false;
    }

    if ((AP_HAL::millis() - this->timeout_begin_ms) > this->timeout_ms){
        return true;
    }else{
        return false;
    }
}

void AP_Vision_Land::init(){
    this->valid = 0;
    this->timeout_begin_ms = 0;
    this->timeout_ms = 5000; //5 sec timeout //TODO: add to params
}

//todo, write reset method. Probably run on takeoff or land completed
//add abort code, redirect to init