#include <AP_HAL/AP_HAL.h>
#include "AP_L1_Control.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_L1_Control::var_info[] = {
    // @Param: PERIOD
    // @DisplayName: L1 control period
    // @Description: Period in seconds of L1 tracking loop. This parameter is the primary control for agressiveness of turns in auto mode. This needs to be larger for less responsive airframes. The default of 20 is quite conservative, but for most RC aircraft will lead to reasonable flight. For smaller more agile aircraft a value closer to 15 is appropriate, or even as low as 10 for some very agile aircraft. When tuning, change this value in small increments, as a value that is much too small (say 5 or 10 below the right value) can lead to very radical turns, and a risk of stalling.
    // @Units: s
    // @Range: 1 60
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PERIOD",    0, AP_L1_Control, _L1_period, 17),

    // @Param: DAMPING
    // @DisplayName: L1 control damping ratio
    // @Description: Damping ratio for L1 control. Increase this in increments of 0.05 if you are getting overshoot in path tracking. You should not need a value below 0.7 or above 0.85.
    // @Range: 0.6 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("DAMPING",   1, AP_L1_Control, _L1_damping, 0.75f),

    // @Param: XTRACK_I
    // @DisplayName: L1 control crosstrack integrator gain
    // @Description: Crosstrack error integrator gain. This gain is applied to the crosstrack error to ensure it converges to zero. Set to zero to disable. Smaller values converge slower, higher values will cause crosstrack error oscillation.
    // @Range: 0 0.1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("XTRACK_I",   2, AP_L1_Control, _L1_xtrack_i_gain, 0.02),

    // @Param: LIM_BANK
    // @DisplayName: Loiter Radius Bank Angle Limit
    // @Description: The sealevel bank angle limit for a continous loiter. (Used to calculate airframe loading limits at higher altitudes). Setting to 0, will instead just scale the loiter radius directly
    // @Units: deg
    // @Range: 0 89
    // @User: Advanced
    AP_GROUPINFO_FRAME("LIM_BANK",   3, AP_L1_Control, _loiter_bank_limit, 0.0f, AP_PARAM_FRAME_PLANE),

       // @Param: AUTO_LIM_BANK
    // @DisplayName:  Bank Angle for calculating turn in point in auto nav
    // @Description: Blank
    // @Units: deg
    // @Range: 0 89
    // @User: Advanced
    AP_GROUPINFO("A_BANK",   4, AP_L1_Control, _auto_bank_limit, 30.0f),
    


   // @Param: Ground_course correction_gain
    // @DisplayName:  integral gain to account for wind speed measurement error
    // @Description: Blank
    // @Units: 1/s
    // @Range: 0 89
    // @User: Advanced
    AP_GROUPINFO("GCC_GAIN", 5, AP_L1_Control, _gcc_gain, 20.0f),

    // @Param: A
	// @DisplayName: Maximum Roll Acceleration
	// @Description: Maximum Acceleration of Roll rate for nav_rll (deg/s/s). 
	// @Range: 1 1000
	// @Increment: 1
	// @User: User
	AP_GROUPINFO("AMAX", 6, AP_L1_Control, _amax, 40),

    // @Param: A
	// @DisplayName: Maximum Roll Acceleration
	// @Description: Maximum Acceleration of Roll rate for nav_rll (deg/s/s). 
	// @Range: 1 1000
	// @Increment: 1
	// @User: User
	AP_GROUPINFO("RMAX", 7, AP_L1_Control, _rmax, 20),


    // @Param: Turn Rate correction factor
	// @DisplayName: Maximum Roll Acceleration
	// @Description: Maximum Acceleration of Roll rate for nav_rll (deg/s/s). 
	// @Range: 1 1000
	// @Increment: 1
	// @User: User
	AP_GROUPINFO("TRCF", 8, AP_L1_Control, _turn_rate_correction_factor,0.625),




    AP_GROUPEND
};

//Bank angle command based on angle between aircraft velocity vector and reference vector to path.
//S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
//Proceedings of the AIAA Guidance, Navigation and Control
//Conference, Aug 2004. AIAA-2004-4900.
//Modified to use PD control for circle tracking to enable loiter radius less than L1 length
//Modified to enable period and damping of guidance loop to be set explicitly
//Modified to provide explicit control over capture angle


/*
  Wrap AHRS yaw if in reverse - radians
 */
float AP_L1_Control::get_yaw()
{
    if (_reverse) {
        return wrap_PI(M_PI + _ahrs.yaw);
    }
    return _ahrs.yaw;
}

/*
  Wrap AHRS yaw sensor if in reverse - centi-degress
 */
int32_t AP_L1_Control::get_yaw_sensor() const
{
    if (_reverse) {
        return wrap_180_cd(18000 + _ahrs.yaw_sensor);
    }
    return _ahrs.yaw_sensor;
}

/*
  return the bank angle needed to achieve tracking from the last
  update_*() operation
 */
int32_t AP_L1_Control::nav_roll_cd() const
{
   
    float ret;
    ret = cosf(_ahrs.pitch)*degrees(atanf(_latAccDem * 0.101972f) * 100.0f); // 0.101972 = 1/9.81
    ret = constrain_float(ret, -9000, 9000);
    return ret;
}


int32_t AP_L1_Control::nav_roll_cd_special() const
{
    
    float bank_limit = DEG_TO_RAD*_auto_bank_limit;
    float roll_rate = DEG_TO_RAD*_rmax; 
    float roll_accel = DEG_TO_RAD*_amax;

    float airspeed = 0; // should set to trim airspeed
    const bool gotAirspeed = _ahrs.airspeed_estimate_true(&airspeed);

    if(!gotAirspeed){
        airspeed = 28.0f; //need to make it a reference to trim speed
    }

    float theta = (bank_limit/2)*((bank_limit/roll_rate)+(roll_rate/roll_accel))*((6*GRAVITY_MSS)/(5*airspeed));
    Vector2f target_ground_velocity =  Vector2f(cosf(_nav_bearing),sinf(_nav_bearing));
    
    Vector2f _windspeed_vector = Vector2f();
    _windspeed_vector.x = _ahrs.wind_estimate().x;
    _windspeed_vector.y = _ahrs.wind_estimate().y;
    Vector2f targer_air_velocity = get_airspeed_from_wind_ground(_windspeed_vector,target_ground_velocity,airspeed);


    float _groundspeed_heading_1 = atan2f(_ahrs.groundspeed_vector().y,_ahrs.groundspeed_vector().x);
    float _ground_turn_angle = wrap_2PI(_nav_bearing-_groundspeed_heading_1+ M_PI)- M_PI;
    float _airspeed_heading_2 = atan2f(targer_air_velocity.y,targer_air_velocity.x);
    float _air_turn_angle = wrap_2PI(_airspeed_heading_2-_ahrs.get_yaw()+ M_PI)- M_PI;
    
     if( _air_turn_angle*_ground_turn_angle<0 && abs(_ground_turn_angle)>M_PI_2 ){
        _air_turn_angle +=   -(abs(_air_turn_angle)/_air_turn_angle)*(M_PI*2);
    }
    
    float bank_angle = constrain_float(((_air_turn_angle+_gcc_integral_sum)/(theta)),-1.0f,1.0f)* _auto_bank_limit;
    

    return (int32_t)(bank_angle*100.0);

    
}





void AP_L1_Control::update_gcc_integrator(){

 
    float ground_track_error = _nav_bearing - atan2f(_ahrs.groundspeed_vector().y,_ahrs.groundspeed_vector().x);

    ground_track_error =  wrap_2PI(ground_track_error+M_PI)-M_PI;

 

    uint32_t now = AP_HAL::micros();
    float dt = (now - _last_nav_angle_update_us) * 1.0e-6f;
    _last_nav_angle_update_us = now;

    if(abs(ground_track_error)<12*DEG_TO_RAD && dt<1.0f){
        _gcc_integral_sum  = _gcc_integral_sum + (ground_track_error *(_gcc_gain/100.0f)*dt);
    }
    else{
         _gcc_integral_sum = 0;
    }
}



/*
  return the lateral acceleration needed to achieve tracking from the last
  update_*() operation
 */
float AP_L1_Control::lateral_acceleration(void) const
{
    return _latAccDem;
}

int32_t AP_L1_Control::nav_bearing_cd(void) const
{
    return wrap_180_cd(RadiansToCentiDegrees(_nav_bearing));
}

int32_t AP_L1_Control::bearing_error_cd(void) const
{
    return RadiansToCentiDegrees(_bearing_error);
}

int32_t AP_L1_Control::target_bearing_cd(void) const
{
    return wrap_180_cd(_target_bearing_cd);
}

/*
  this is the turn distance assuming a 90 degree turn
 */
float AP_L1_Control::turn_distance(float wp_radius) const
{
    wp_radius *= sq(_ahrs.get_EAS2TAS());
    
    return MIN(wp_radius, _L1_dist);

    //return MIN(wp_radius,)
}

/*
  this approximates the turn distance for a given turn angle. If the
  turn_angle is > 90 then a 90 degree turn distance is used, otherwise
  the turn distance is reduced linearly.
  This function allows straight ahead mission legs to avoid thinking
  they have reached the waypoint early, which makes things like camera
  trigger and ball drop at exact positions under mission control much easier
 */
float AP_L1_Control::turn_distance(float groundspeed, float turn_angle) const
{
    float turnRadius= groundspeed*groundspeed/(10*tanf(radians(_auto_bank_limit)));
    float returnValue =turnRadius/tanf(radians((180.0f-abs(turn_angle))/2));
    return returnValue;
}


float AP_L1_Control::turn_distance_special( const struct Location &current_loc, const struct Location &turn_WP, const struct Location &next_WP, const float roll_rate_deg, const float roll_accel_deg) const
{
   
    Vector2f _groundspeed_vector_1 = current_loc.get_distance_NE(turn_WP) ;
    Vector2f _windspeed_vector = Vector2f();
    _windspeed_vector.x = _ahrs.wind_estimate().x;
    _windspeed_vector.y = _ahrs.wind_estimate().y;
    _groundspeed_vector_1 = _groundspeed_vector_1.normalized()*_ahrs.groundspeed();

    Vector2f _airspeed_vector_1 = _groundspeed_vector_1 - _windspeed_vector;
    Vector2f _groundspeed_vector_2 = turn_WP.get_distance_NE(next_WP);
    float airspeed = 0; // should set to trim airspeed
    
    const bool gotAirspeed = _ahrs.airspeed_estimate_true(&airspeed);
    if(!gotAirspeed){
        airspeed = 28.0f; //need to make it a reference to trim speed
    }

    Vector2f _airspeed_vector_2 = get_airspeed_from_wind_ground(_windspeed_vector, _groundspeed_vector_2, airspeed );

    float bank_limit = DEG_TO_RAD*_auto_bank_limit;
    float roll_rate = DEG_TO_RAD*roll_rate_deg;
    float roll_accel = DEG_TO_RAD*roll_accel_deg;

    //catches case where we have to turn more than 180 degrees in air frame
 
    float _groundspeed_heading_1 = atan2f(_groundspeed_vector_1.y,_groundspeed_vector_1.x);
    float _groundspeed_heading_2 = atan2f(_groundspeed_vector_2.y,_groundspeed_vector_2.x);
    float _ground_turn_angle = wrap_2PI(_groundspeed_heading_2-_groundspeed_heading_1+ M_PI)- M_PI;
    float _airspeed_heading_1 = atan2f(_airspeed_vector_1.y,_airspeed_vector_1.x);
    float _airspeed_heading_2 = atan2f(_airspeed_vector_2.y,_airspeed_vector_2.x);
    float _air_turn_angle = wrap_2PI(_airspeed_heading_2-_airspeed_heading_1+ M_PI)- M_PI;

    if( _air_turn_angle*_ground_turn_angle<0 ){
        _air_turn_angle +=   -(abs(_air_turn_angle)/_air_turn_angle)*(M_PI*2);
    }

    float theta = _turn_rate_correction_factor*(bank_limit/2)*((bank_limit/roll_rate)+(roll_rate/roll_accel))*((6*GRAVITY_MSS)/(5*airspeed));//check this, seems too big

    if(abs(_air_turn_angle) <  2*theta){
        float turnRadius= sq(_ahrs.groundspeed_vector().length())/(GRAVITY_MSS*tanf(bank_limit/2));
        return turnRadius/tanf(radians((180.0f-abs(RAD_TO_DEG* _airspeed_vector_1.angle(_airspeed_vector_2)))/2));  
    }
    
    Vector2f _airspeed_vector_1_normalized = _airspeed_vector_1.normalized();
    Vector2f perp_airspeed_vector_1 = Vector2f();
    perp_airspeed_vector_1.x = -_airspeed_vector_1_normalized.y;
    perp_airspeed_vector_1.y = _airspeed_vector_1_normalized.x;
    if(_air_turn_angle<0.0f){
        perp_airspeed_vector_1 = -perp_airspeed_vector_1;
    }
          
    float beta = abs(_air_turn_angle);
    Vector2f turnDistanceXY = ( (perp_airspeed_vector_1* (2- cosf(theta) + cosf(beta-theta) -(2*cosf(beta)))) + (_airspeed_vector_1_normalized*(sinf(theta)-sinf(beta-theta)+(2*sinf(beta)))) );
    turnDistanceXY = turnDistanceXY*((5*sq(airspeed))/(6*GRAVITY_MSS*bank_limit*_turn_rate_correction_factor));


    float turnTimealphaPortion = ((5*airspeed)/(6*GRAVITY_MSS*bank_limit*_turn_rate_correction_factor))*(abs(_air_turn_angle)-(2*theta));
    float turnTimeThetaPortion = (2*((bank_limit/roll_rate)+(roll_rate/roll_accel)));
    float turnTime = turnTimealphaPortion +turnTimeThetaPortion;

    turnDistanceXY = turnDistanceXY+ (_windspeed_vector*turnTime);

    Vector2f turnDistanceParallel = turnDistanceXY;
    turnDistanceParallel =   _groundspeed_vector_1 * (turnDistanceParallel *  _groundspeed_vector_1)/( _groundspeed_vector_1* _groundspeed_vector_1);     

    Vector2f turnDistancePerpendicular = turnDistanceXY -turnDistanceParallel;

    Vector2f _groundDistance_2 = _groundspeed_vector_2.normalized()*(turnDistancePerpendicular.length()/sinf(_groundspeed_vector_1.angle(_groundspeed_vector_2)));

    Vector2f _groundDistance_1 = turnDistanceXY - _groundDistance_2;

    return _groundDistance_1.length();

}

Vector2f AP_L1_Control::get_airspeed_from_wind_ground(const Vector2f wind, const Vector2f ground, const float airspeed) const
{
    
    Vector2f G = ground.normalized();
    Vector2f WindInTrack = wind;
    WindInTrack = ground * (WindInTrack * ground)/(ground*ground);       
    /*bool flying_backwards = false;
    if(WindInTrack.length()>airspeed){
        flying_backwards = true;
    }
    */
    Vector2f C = WindInTrack - wind;

    float magC = C.length();

    Vector2f A = Vector2f();

    if(airspeed>magC){
        A = G*sqrt( sq(airspeed)- sq(magC)) +C;
    }
    else{
        A = -wind.normalized()*airspeed;
    }
    return A;


}


float AP_L1_Control::loiter_radius(const float radius) const
{
    // prevent an insane loiter bank limit
    float sanitized_bank_limit = constrain_float(_loiter_bank_limit, 0.0f, 89.0f);
    float lateral_accel_sea_level = tanf(radians(sanitized_bank_limit)) * GRAVITY_MSS;

    float nominal_velocity_sea_level;
    if(_spdHgtControl == nullptr) {
        nominal_velocity_sea_level = 0.0f;
    } else {
        nominal_velocity_sea_level =  _spdHgtControl->get_target_airspeed();
    }

    float eas2tas_sq = sq(_ahrs.get_EAS2TAS());

    if (is_zero(sanitized_bank_limit) || is_zero(nominal_velocity_sea_level) ||
        is_zero(lateral_accel_sea_level)) {
        // Missing a sane input for calculating the limit, or the user has
        // requested a straight scaling with altitude. This will always vary
        // with the current altitude, but will at least protect the airframe
        return radius * eas2tas_sq;
    } else {
        float sea_level_radius = sq(nominal_velocity_sea_level) / lateral_accel_sea_level;
        if (sea_level_radius > radius) {
            // If we've told the plane that its sea level radius is unachievable fallback to
            // straight altitude scaling
            return radius * eas2tas_sq;
        } else {
            // select the requested radius, or the required altitude scale, whichever is safer
            return MAX(sea_level_radius * eas2tas_sq, radius);
        }
    }
}

bool AP_L1_Control::reached_loiter_target(void)
{
    return _WPcircle;
}

/**
   prevent indecision in our turning by using our previous turn
   decision if we are in a narrow angle band pointing away from the
   target and the turn angle has changed sign
 */
void AP_L1_Control::_prevent_indecision(float &Nu)
{
    const float Nu_limit = 0.9f*M_PI;
    if (fabsf(Nu) > Nu_limit &&
        fabsf(_last_Nu) > Nu_limit &&
        labs(wrap_180_cd(_target_bearing_cd - get_yaw_sensor())) > 12000 &&
        Nu * _last_Nu < 0.0f) {
        // we are moving away from the target waypoint and pointing
        // away from the waypoint (not flying backwards). The sign
        // of Nu has also changed, which means we are
        // oscillating in our decision about which way to go
        Nu = _last_Nu;
    }
}

// update L1 control for waypoint navigation
void AP_L1_Control::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min)
{

    struct Location _current_loc;
    float Nu;
    float xtrackVel;
    float ltrackVel;

    uint32_t now = AP_HAL::micros();
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f;
    if (dt > 0.1) {
        dt = 0.1;
        _L1_xtrack_i = 0.0f;
    }
    _last_update_waypoint_us = now;

    // Calculate L1 gain required for specified damping
    float K_L1 = 4.0f * _L1_damping * _L1_damping;

    // Get current position and velocity
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    // update _target_bearing_cd
    _target_bearing_cd = _current_loc.get_bearing_to(next_WP);

    //Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();
    if (groundSpeed < 0.1f) {
        // use a small ground speed vector in the right direction,
        // allowing us to use the compass heading at zero GPS velocity
        groundSpeed = 0.1f;
        _groundspeed_vector = Vector2f(cosf(get_yaw()), sinf(get_yaw())) * groundSpeed;
    }

    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period
    // 0.3183099 = 1/1/pipi
    _L1_dist = MAX(0.3183099f * _L1_damping * _L1_period * groundSpeed, dist_min);

    // Calculate the NE position of WP B relative to WP A
    Vector2f AB = prev_WP.get_distance_NE(next_WP);
    float AB_length = AB.length();

    // Check for AB zero length and track directly to the destination
    // if too small
    if (AB.length() < 1.0e-6f) {
        AB = _current_loc.get_distance_NE(next_WP);
        if (AB.length() < 1.0e-6f) {
            AB = Vector2f(cosf(get_yaw()), sinf(get_yaw()));
        }
    }
    AB.normalize();

    // Calculate the NE position of the aircraft relative to WP A
    const Vector2f A_air = prev_WP.get_distance_NE(_current_loc);

    // calculate distance to target track, for reporting
    _crosstrack_error = A_air % AB;

    //Determine if the aircraft is behind a +-135 degree degree arc centred on WP A
    //and further than L1 distance from WP A. Then use WP A as the L1 reference point
    //Otherwise do normal L1 guidance
    float WP_A_dist = A_air.length();
    float alongTrackDist = A_air * AB;
    if (WP_A_dist > _L1_dist && alongTrackDist/MAX(WP_A_dist, 1.0f) < -0.7071f)
    {
        //Calc Nu to fly To WP A
        Vector2f A_air_unit = (A_air).normalized(); // Unit vector from WP A to aircraft
        xtrackVel = _groundspeed_vector % (-A_air_unit); // Velocity across line
        ltrackVel = _groundspeed_vector * (-A_air_unit); // Velocity along line
        Nu = atan2f(xtrackVel,ltrackVel);
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
    } else if (alongTrackDist > AB_length + groundSpeed*3) {
        // we have passed point B by 3 seconds. Head towards B
        // Calc Nu to fly To WP B
        const Vector2f B_air = next_WP.get_distance_NE(_current_loc);
        Vector2f B_air_unit = (B_air).normalized(); // Unit vector from WP B to aircraft
        xtrackVel = _groundspeed_vector % (-B_air_unit); // Velocity across line
        ltrackVel = _groundspeed_vector * (-B_air_unit); // Velocity along line
        Nu = atan2f(xtrackVel,ltrackVel);
        _nav_bearing = atan2f(-B_air_unit.y , -B_air_unit.x); // bearing (radians) from AC to L1 point
    } else { //Calc Nu to fly along AB line

        //Calculate Nu2 angle (angle of velocity vector relative to line connecting waypoints)
        xtrackVel = _groundspeed_vector % AB; // Velocity cross track
        ltrackVel = _groundspeed_vector * AB; // Velocity along track
        float Nu2 = atan2f(xtrackVel,ltrackVel);
        //Calculate Nu1 angle (Angle to L1 reference point)
        float sine_Nu1 = _crosstrack_error/MAX(_L1_dist, 0.1f);
        //Limit sine of Nu1 to provide a controlled track capture angle of 45 deg
        sine_Nu1 = constrain_float(sine_Nu1, -0.7071f, 0.7071f);
        float Nu1 = asinf(sine_Nu1);

        // compute integral error component to converge to a crosstrack of zero when traveling
        // straight but reset it when disabled or if it changes. That allows for much easier
        // tuning by having it re-converge each time it changes.
        if (_L1_xtrack_i_gain <= 0 || !is_equal(_L1_xtrack_i_gain.get(), _L1_xtrack_i_gain_prev)) {
            _L1_xtrack_i = 0;
            _L1_xtrack_i_gain_prev = _L1_xtrack_i_gain;
        } else if (fabsf(Nu1) < radians(5)) {
            _L1_xtrack_i += Nu1 * _L1_xtrack_i_gain * dt;

            // an AHRS_TRIM_X=0.1 will drift to about 0.08 so 0.1 is a good worst-case to clip at
            _L1_xtrack_i = constrain_float(_L1_xtrack_i, -0.1f, 0.1f);
        }

        // to converge to zero we must push Nu1 harder
        Nu1 += _L1_xtrack_i;

        Nu = Nu1 + Nu2;
        _nav_bearing = atan2f(AB.y, AB.x) + Nu1; // bearing (radians) from AC to L1 point
    }

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    //Limit Nu to +-(pi/2)
    Nu = constrain_float(Nu, -1.5708f, +1.5708f);
    _latAccDem = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);
    // sure we can work out a better algorithm for this


    // Waypoint capture status is always false during waypoint following
    _WPcircle = false;

    _bearing_error = Nu; // bearing error angle (radians), +ve to left of track

    _data_is_stale = false; // status are correctly updated with current waypoint data


    update_gcc_integrator();

}

// update L1 control for loitering
void AP_L1_Control::update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction)
{
    struct Location _current_loc;

    // scale loiter radius with square of EAS2TAS to allow us to stay
    // stable at high altitude
    radius = loiter_radius(fabsf(radius));

    // Calculate guidance gains used by PD loop (used during circle tracking)
    float omega = (6.2832f / _L1_period);
    float Kx = omega * omega;
    float Kv = 2.0f * _L1_damping * omega;

    // Calculate L1 gain required for specified damping (used during waypoint capture)
    float K_L1 = 4.0f * _L1_damping * _L1_damping;

    //Get current position and velocity
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    //Calculate groundspeed
    float groundSpeed = MAX(_groundspeed_vector.length() , 1.0f);


    // update _target_bearing_cd
    _target_bearing_cd = _current_loc.get_bearing_to(center_WP);


    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period
    // 0.3183099 = 1/pi
    _L1_dist = 0.3183099f * _L1_damping * _L1_period * groundSpeed;

    //Calculate the NE position of the aircraft relative to WP A
    const Vector2f A_air = center_WP.get_distance_NE(_current_loc);

    // Calculate the unit vector from WP A to aircraft
    // protect against being on the waypoint and having zero velocity
    // if too close to the waypoint, use the velocity vector
    // if the velocity vector is too small, use the heading vector
    Vector2f A_air_unit;
    if (A_air.length() > 0.1f) {
        A_air_unit = A_air.normalized();
    } else {
        if (_groundspeed_vector.length() < 0.1f) {
            A_air_unit = Vector2f(cosf(_ahrs.yaw), sinf(_ahrs.yaw));
        } else {
            A_air_unit = _groundspeed_vector.normalized();
        }
    }

    //Calculate Nu to capture center_WP
    float xtrackVelCap = A_air_unit % _groundspeed_vector; // Velocity across line - perpendicular to radial inbound to WP
    float ltrackVelCap = - (_groundspeed_vector * A_air_unit); // Velocity along line - radial inbound to WP
    float Nu = atan2f(xtrackVelCap,ltrackVelCap);

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    Nu = constrain_float(Nu, -M_PI_2, M_PI_2); //Limit Nu to +- Pi/2

    //Calculate lat accln demand to capture center_WP (use L1 guidance law)
    float latAccDemCap = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);

    //Calculate radial position and velocity errors
    float xtrackVelCirc = -ltrackVelCap; // Radial outbound velocity - reuse previous radial inbound velocity
    float xtrackErrCirc = A_air.length() - radius; // Radial distance from the loiter circle

    // keep crosstrack error for reporting
    _crosstrack_error = xtrackErrCirc;

    //Calculate PD control correction to circle waypoint_ahrs.roll
    float latAccDemCircPD = (xtrackErrCirc * Kx + xtrackVelCirc * Kv);

    //Calculate tangential velocity
    float velTangent = xtrackVelCap * float(loiter_direction);

    //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
    if (ltrackVelCap < 0.0f && velTangent < 0.0f) {
        latAccDemCircPD =  MAX(latAccDemCircPD, 0.0f);
    }

    // Calculate centripetal acceleration demand
    float latAccDemCircCtr = velTangent * velTangent / MAX((0.5f * radius), (radius + xtrackErrCirc));

    //Sum PD control and centripetal acceleration to calculate lateral manoeuvre demand
    float latAccDemCirc = loiter_direction * (latAccDemCircPD + latAccDemCircCtr);

    // Perform switchover between 'capture' and 'circle' modes at the
    // point where the commands cross over to achieve a seamless transfer
    // Only fly 'capture' mode if outside the circle
    if (xtrackErrCirc > 0.0f && loiter_direction * latAccDemCap < loiter_direction * latAccDemCirc) {
        _latAccDem = latAccDemCap;
        _WPcircle = false;
        _bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
    } else {
        _latAccDem = latAccDemCirc;
        _WPcircle = true;
        _bearing_error = 0.0f; // bearing error (radians), +ve to left of track
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians)from AC to L1 point
    }

    _data_is_stale = false; // status are correctly updated with current waypoint data
}


// update L1 control for heading hold navigation
void AP_L1_Control::update_heading_hold(int32_t navigation_heading_cd)
{
    // Calculate normalised frequency for tracking loop
    const float omegaA = 4.4428f/_L1_period; // sqrt(2)*pi/period
    // Calculate additional damping gain

    int32_t Nu_cd;
    float Nu;

    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = wrap_180_cd(navigation_heading_cd);
    _nav_bearing = radians(navigation_heading_cd * 0.01f);

    Nu_cd = _target_bearing_cd - wrap_180_cd(_ahrs.yaw_sensor);
    Nu_cd = wrap_180_cd(Nu_cd);
    Nu = radians(Nu_cd * 0.01f);

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    //Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();

    // Calculate time varying control parameters
    _L1_dist = groundSpeed / omegaA; // L1 distance is adjusted to maintain a constant tracking loop frequency
    float VomegaA = groundSpeed * omegaA;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;

    _crosstrack_error = 0;

    _bearing_error = Nu; // bearing error angle (radians), +ve to left of track

    // Limit Nu to +-pi
    Nu = constrain_float(Nu, -M_PI_2, M_PI_2);
    _latAccDem = 2.0f*sinf(Nu)*VomegaA;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

// update L1 control for level flight on current heading
void AP_L1_Control::update_level_flight(void)
{
    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = _ahrs.yaw_sensor;
    _nav_bearing = _ahrs.yaw;
    _bearing_error = 0;
    _crosstrack_error = 0;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;

    _latAccDem = 0;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}
