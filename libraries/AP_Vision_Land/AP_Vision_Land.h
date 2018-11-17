// #include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/AP_Common.h>
#include <stdint.h>

class AP_Vision_Land
{
public:
    // Constructor
    AP_Vision_Land();

    void update_vision_land();

    // process a LANDING_TARGET mavlink message
    void handle_msg(mavlink_message_t* msg);

}