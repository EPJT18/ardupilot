// #include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/AP_Common.h>
#include <stdint.h>

class AP_Vision_Land
{
public:
    // void update_vision_land();

    // process a LANDING_TARGET mavlink message
    void handle_msg(mavlink_message_t* msg);
    int ok();

    float get_target_lat();
    float get_target_lng();

private:
    uint64_t    timestamp_us;  // timestamp from message
    uint8_t     frame;         // Frame of message
    float       x, y;         // Target location
    uint8_t     valid;         // Target position valid
};
