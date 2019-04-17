#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <stdint.h>

class AP_Vision_Land
{
public:
    uint8_t     active;                 // vision landing system is active

    AP_Vision_Land(const AP_AHRS_NavEKF& ahrs);
    void init();
    void handle_msg(mavlink_message_t* msg);
    int ok();
    bool waypoint_injection_check();
    int waypoint_injected();
    float get_target_lat();
    float get_target_lng();
    bool search_timeout();
    Location inject_updated_waypoint(Location next_loc);
    bool get_relative_cms(Vector2f& v);
    void send_begin_request(mavlink_channel_t chan);

private:
    uint64_t    timestamp_us;           // timestamp from message
    uint8_t     frame;                  // Frame of message
    float       x, y;                   // Target location
    uint8_t     valid;                  // Target position valid
    int         have_injected;          // tells if vision landing waypoint is being used
    uint32_t    timeout_begin_ms;       // Unix time ms, search commenced
    uint32_t    timeout_ms;             // Timeout for searching for target, resume programmed wp
    uint32_t    last_wp_ms;             // Time when last waypoint was sent, for regulating rate of WP update
    uint32_t    wp_update_period;       // Waypoint update period, slow rate avoids sudden movements
    
    // references to inertial nav and ahrs libraries
    const AP_AHRS_NavEKF&       _ahrs;
};
