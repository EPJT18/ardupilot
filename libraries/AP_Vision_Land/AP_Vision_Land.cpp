#include "AP_Vision_Land.h"



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

int ok(){
    return this.valid;
}

//may need to add 'PACKED' (AP_Common.h, l 133)
struct Location get_target_location(){
    struct Location l;
    l.lat = this.x
    l.lng = this.y
}

