//
// functions to support precision landing
//

#include "Plane.h"

#if PRECISION_LANDING == ENABLED
void Plane::init_precland()
{
    plane.g2.precland.init(scheduler.get_loop_rate_hz());
}

void Plane::update_precland()
{
    float hagl = plane.relative_ground_altitude(plane.g.rangefinder_landing);
    g2.precland.update(hagl*100, rangefinder_alt_ok());
}
#endif
