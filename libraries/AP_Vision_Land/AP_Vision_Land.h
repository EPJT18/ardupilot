// #include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/AP_Common.h>
#include <stdint.h>

class AP_Vision_Land
{
public:
    // void update_vision_land();
    AP_Vision_Land();
    void init();
    // process a LANDING_TARGET mavlink message
    void handle_msg(mavlink_message_t* msg);
    int ok();
    int waypoint_injected();
    float get_target_lat();
    float get_target_lng();
    bool search_timeout();
    Location inject_updated_waypoint(Location next_loc);

private:
    uint64_t    timestamp_us;           // timestamp from message
    uint8_t     frame;                  // Frame of message
    float       x, y;                   // Target location
    uint8_t     valid;                  // Target position valid
    int         have_injected;          // tells if vision landing waypoint is being used
    uint32_t    timeout_begin_ms;       // Unix time ms, search commenced
    uint32_t    timeout_ms;             // Timeout for searching for target, resume programmed wp
};
