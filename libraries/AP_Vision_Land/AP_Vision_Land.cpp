#include "AP_Vision_Land.h"

AP_Vision_Land::AP_Vision_Land(){
    // dummy data to test landing
    this->x = -353630116;
    this->y = 1491654995;
    this->valid = 1;
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
    Location new_loc = next_loc;
    new_loc.lat = this->x;
    new_loc.lng = this->y;
    return new_loc;
}