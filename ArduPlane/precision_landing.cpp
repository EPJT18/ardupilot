//
// functions to support precision landing
//

#include "Plane.h"

#if PRECISION_LANDING == ENABLED

void Plane::init_precland()
{
    plane.precland.init();
}

void Plane::update_precland()
{
    int32_t height_above_ground_cm = current_loc.alt;
    //TODO make this functional
    // use range finder altitude if it is valid, else try to get terrain alt
    // if (rangefinder_alt_ok()) {
        height_above_ground_cm = rangefinder_state.height_estimate;
    // } else if (terrain_use()) {
    //     if (!current_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, height_above_ground_cm)) {
    //         height_above_ground_cm = current_loc.alt;
    //     }
    // }

    plane.precland.update(height_above_ground_cm, rangefinder_alt_ok());
}
#endif
