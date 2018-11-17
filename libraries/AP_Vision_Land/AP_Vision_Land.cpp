#include "AP_Vision_Land.h"



// handle_msg - Process a LANDING_TARGET mavlink message
void AP_Vision_Land::handle_msg(mavlink_message_t* msg)
{
        // parse mavlink message
    __mavlink_landing_target_t packet;
    mavlink_msg_landing_target_decode(msg, &packet);

    _timestamp_us = packet.time_usec;       // Unix time

    _ref_frame = packet.frame;              // Frame of target pos reference

    // Position of the target in _ref_frame Frame
    _pos_x = packet.x;
    _pos_y = packet.y;
    // _pos_z = packet.z; //shouldn't need alt

    _valid = packet.position_valid;         // Is the position valid 0=False, 1=True

}

void update_vision_land(){
    // handle mavlink
    this.handle_msg();
}