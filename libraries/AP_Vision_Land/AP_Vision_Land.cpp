#include "AP_Vision_Land.h"
#include <DataFlash/DataFlash.h>

AP_Vision_Land::AP_Vision_Land(const AP_AHRS_NavEKF& ahrs):
    _ahrs(ahrs)
{
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

    // Add to log
    DataFlash_Class::instance()->Log_Write("VLND","Lat,Lng,Valid","ffB", 
        (double)this->x,
        (double)this->y,
        this->valid);

    if(this->valid){
        this->timeout_begin_ms = AP_HAL::millis();
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

bool AP_Vision_Land::waypoint_injection_check(){

    if (AP_HAL::millis()-this->last_wp_ms > this->wp_update_period){
        this->last_wp_ms = AP_HAL::millis();
        return true;
    }
    return false;
}

Location AP_Vision_Land::inject_updated_waypoint(Location next_loc){
    this->have_injected = 1;
    Location new_loc = next_loc;
    new_loc.lat = this->x;
    new_loc.lng = this->y;
    return new_loc;
}

bool AP_Vision_Land::get_relative_cms(Vector2f& v){
    Location pad_loc, tmp_loc;
    pad_loc.lat = this->x;
    pad_loc.lng = this->y;
    if(!_ahrs.get_location(tmp_loc)){
        return false;
    }
    v = location_diff(tmp_loc, pad_loc) * 100.0f;
    return true;
}

int AP_Vision_Land::waypoint_injected(){
    this->timeout_begin_ms = AP_HAL::millis(); //reset timeout
    return this->have_injected;
}

bool AP_Vision_Land::search_timeout(){

    if ((AP_HAL::millis() - this->timeout_begin_ms) > this->timeout_ms){
        return true;
    }else{
        return false;
    }
}

void AP_Vision_Land::init(){
    this->valid = 0;
    this->timeout_begin_ms = AP_HAL::millis();
    this->last_wp_ms = AP_HAL::millis();
    this->timeout_ms = 5000; //5 sec timeout //TODO: add to params
    this->active = 0;
    this->wp_update_period = 2000; //TODO: add to params
}



//todo, write reset method. Probably run on takeoff or land completed
//add abort code, redirect to init