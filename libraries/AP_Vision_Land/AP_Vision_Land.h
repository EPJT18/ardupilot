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

private:
    uint64_t    _timestamp_us;  // timestamp from message
    uint8_t     _frame;         // Frame of message
    float       _x, _y;         // Target location
    uint8_t     _valid;         // Target position valid
}