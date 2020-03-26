#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AC_PrecLand.h"
#include "AC_PrecLand_Backend.h"
#include "AC_PrecLand_Companion.h"
#include "AC_PrecLand_IRLock.h"
#include "AC_PrecLand_SITL_Gazebo.h"
#include "AC_PrecLand_SITL.h"

#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_PrecLand::var_info[] = {
    // @Param: ENABLED
    // @DisplayName: Precision Land enabled/disabled and behaviour
    // @Description: Precision Land enabled/disabled and behaviour
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLED", 0, AC_PrecLand, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TYPE
    // @DisplayName: Precision Land Type
    // @Description: Precision Land Type
    // @Values: 0:None, 1:CompanionComputer, 2:IRLock, 3:SITL_Gazebo, 4:SITL
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TYPE",    1, AC_PrecLand, _type, 0),

    // @Param: LAND_OFS_X
    // @DisplayName: Land offset forward
    // @Description: Desired landing position of the camera forward of the target in vehicle body frame
    // @Range: -20 20
    // @Increment: 1
    // @User: Advanced
    // @Units: cm
    AP_GROUPINFO("LAND_OFS_X",    3, AC_PrecLand, _land_ofs_cm_x, 0),

    // @Param: LAND_OFS_Y
    // @DisplayName: Land offset right
    // @Description: desired landing position of the camera right of the target in vehicle body frame
    // @Range: -20 20
    // @Increment: 1
    // @User: Advanced
    // @Units: cm
    AP_GROUPINFO("LAND_OFS_Y",    4, AC_PrecLand, _land_ofs_cm_y, 0),

    // @Param: EST_TYPE
    // @DisplayName: Precision Land Estimator Type
    // @Description: Specifies the estimation method to be used
    // @Values: 0:RawSensor, 1:KalmanFilter, 2:SwoopFilter
    // @User: Advanced
    AP_GROUPINFO("EST_TYPE",    5, AC_PrecLand, _estimator_type, 2),

    // @Param: ACC_P_NSE
    // @DisplayName: Kalman Filter Accelerometer Noise
    // @Description: Kalman Filter Accelerometer Noise, higher values weight the input from the camera more, accels less
    // @Range: 0.5 5
    // @User: Advanced
    AP_GROUPINFO("ACC_P_NSE", 6, AC_PrecLand, _accel_noise, 2.5f),

    // @Param: CAM_ANG_X
    // @DisplayName: Camera X angle offset
    // @Description: X angle of the camera in body frame. Right hand rule.
    // @Range: 0 36000
    // @Increment: 1
    // @Units: cdeg
    // @User: Advanced

    // @Param: CAM_ANG_Y
    // @DisplayName: Camera Y angle offset
    // @Description: Y angle of the camera in body frame. Right hand rule.
    // @Range: 0 36000
    // @Increment: 1
    // @Units: cdeg
    // @User: Advanced

    // @Param: CAM_ANG_Z
    // @DisplayName: Camera Z angle offset
    // @Description: Z angle of the camera in body frame. Right hand rule.
    // @Range: 0 36000
    // @Increment: 1
    // @Units: cdeg
    // @User: Advanced
    AP_GROUPINFO("CAM_ANG", 7, AC_PrecLand, _cam_offset_ang, 0.0f),

    // @Param: CAM_POS_X
    // @DisplayName: Camera X position offset
    // @Description: X position of the camera in body frame. Positive X is forward of the origin.
    // @Units: m
    // @User: Advanced

    // @Param: CAM_POS_Y
    // @DisplayName: Camera Y position offset
    // @Description: Y position of the camera in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @User: Advanced

    // @Param: CAM_POS_Z
    // @DisplayName: Camera Z position offset
    // @Description: Z position of the camera in body frame. Positive Z is down from the origin.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("CAM_POS", 8, AC_PrecLand, _cam_offset, 0.0f),

    // @Param: BUS
    // @DisplayName: Sensor Bus
    // @Description: Precland sensor bus for I2C sensors.
    // @Values: -1:DefaultBus,0:InternalI2C,1:ExternalI2C
    // @User: Advanced
    AP_GROUPINFO("BUS",    9, AC_PrecLand, _bus, -1),

    // @Param: LAG
    // @DisplayName: Precision Landing sensor lag
    // @Description: Precision Landing sensor lag, to cope with variable landing_target latency
    // @Range: 0.02 0.75
    // @Increment: 1
    // @Units: s
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("LAG", 10, AC_PrecLand, _lag, 0.2f), // 20ms is the old default buffer size (8 frames @ 400hz/2.5ms)

    // @Param: TACQ_TIM
    // @DisplayName: Target Aquired Timeout
    // @Description: Time until estimate resets if target not seen
    // @Range: 2 20
    // @Increment: 1
    // @Units: s
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TACQ_TIM", 11, AC_PrecLand, _tacq_timeout, 2),

    // @Param: NUM_SMP
    // @DisplayName: Buffer size of swoop filter
    // @Description: Buffer size of swoop filter
    // @Range: 10 200
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("NUM_SMP", 12, AC_PrecLand, _num_ave_samples, 10),

    // @Param: TIMEOUT
    // @DisplayName: Precision Landing timeout
    // @Description: How long to wait to capture target at MIN_SRC_ALT before giving up. 
    // @Range: 0 60
    // @Increment: 1
    // @Units: s
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TIMEOUT", 13, AC_PrecLand, _timeout, 5),

    // @Param: MAX_POS_ERR
    // @DisplayName: Acceptable pos error
    // @Description: Will descend at full speed while within this radius of the acceptable error, else will scale descent speed accordingly
    // @Units: cm
    // @Range: 10 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX_POS_ERR", 15, AC_PrecLand, _acceptable_error_cm, 100),

    // @Param: MIN_SRC_ALT
    // @DisplayName: Minimum Search Alt
    // @Description: If the target has not been seen and the vehicle has descended to this alt, stop and search for PLND_TACQ_TIM seconds
    // @Units: m
    // @Range: 10 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MIN_SRC_ALT", 16, AC_PrecLand, _min_search_alt, 40),

    // @Param: MIN_ABRT_ALT
    // @DisplayName: Minimum Abort Alt
    // @Description: If the target is lost (ie PLND_TACQ_TIM expires) below this alt, continue with last target position estimate. Else abort. 
    // @Units: m
    // @Range: 10 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MIN_ABT_ALT", 17, AC_PrecLand, _min_abort_alt, 15),

    // @Param: MAX_TGT_ERR
    // @DisplayName: Acceptable target estimate error
    // @Description: Max error the estimate buffer can have before becoming invalid.
    // @Units: cm
    // @Range: 10 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MAX_TGT_ERR", 18, AC_PrecLand, _acceptable_target_error_cm, 100),

    // @Param: MAX_PCT_OTL
    // @DisplayName: Acceptable outlier cull percentage
    // @Description: Max percentage of the buffer that are considered outliers before the estimate becomes invalid
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MAX_PCT_OTL", 19, AC_PrecLand, _max_cull_pct, 0.3),

    // @Param: MAX_TGT_DST
    // @DisplayName: Max Target Distance
    // @Description: Maximum distance a target can be from the vehicle and be accepted (beware includes height)
    // @Units: m
    // @Range: 0 200
    // @Increment: 0.01
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MAX_TGT_DST", 20, AC_PrecLand, _max_target_distance, 10),



    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PrecLand::AC_PrecLand()
{
    // set parameters to defaults
    AP_Param::setup_object_defaults(this, var_info);
}

// perform any required initialisation of landing controllers
// update_rate_hz should be the rate at which the update method will be called in hz
void AC_PrecLand::init(uint16_t update_rate_hz)
{
    // exit immediately if init has already been run
    if (_backend != nullptr) {
        return;
    }

    // default health to false
    _backend = nullptr;
    _backend_state.healthy = false;

    // create inertial history buffer
    // constrain lag parameter to be within bounds
    _lag = constrain_float(_lag, 0.02f, 0.75f);

    // calculate inertial buffer size from lag and minimum of main loop rate and update_rate_hz argument
    const uint16_t inertial_buffer_size = MAX((uint16_t)roundf(_lag * MIN(update_rate_hz, AP::scheduler().get_loop_rate_hz())), 1);

    // instantiate ring buffer to hold inertial history, return on failure so no backends are created
    _inertial_history = new ObjectArray<inertial_data_frame_s>(inertial_buffer_size);
    if (_inertial_history == nullptr) {
        return;
    }

    // create a history of past estimates, return on failure so no backends are created
    _target_history = new ObjectArray<Vector2f>(_num_ave_samples);
    if (_target_history == nullptr) {
        return;
    }

    // initialise timeout and filter
    reinit();

    // instantiate backend based on type parameter
    switch ((enum PrecLandType)(_type.get())) {
        // no type defined
        case PRECLAND_TYPE_NONE:
        default:
            return;
        // companion computer
        case PRECLAND_TYPE_COMPANION:
            _backend = new AC_PrecLand_Companion(*this, _backend_state);
            break;
        // IR Lock
        case PRECLAND_TYPE_IRLOCK:
            _backend = new AC_PrecLand_IRLock(*this, _backend_state);
            break;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        case PRECLAND_TYPE_SITL_GAZEBO:
            _backend = new AC_PrecLand_SITL_Gazebo(*this, _backend_state);
            break;
        case PRECLAND_TYPE_SITL:
            _backend = new AC_PrecLand_SITL(*this, _backend_state);
            break;
#endif
    }

    // init backend
    if (_backend != nullptr) {
        _backend->init();
    }
}

void AC_PrecLand::reinit(void){

    //timeout
    _commence_time = 0; // if zero, not active
    reset_swoop_filter();
    
}

void AC_PrecLand::start_search_timer(void){
    _commence_time = AP_HAL::millis();
}

void AC_PrecLand::reset_swoop_filter(void){
    //swoop filter
    _outliers=0;
    _swoop_has_been_confident = false;
    _update_swoop_filt = false;
    _target_history->clear();
}

bool AC_PrecLand::swoop_filter_ready(void){

    if(_estimator_type==ESTIMATOR_TYPE_SWOOP_FILTER && (_target_history->space()>0)){
        return false;
    }else{
        return true;
    }
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand::update(float rangefinder_alt_cm, bool rangefinder_alt_valid)
{
    // exit immediately if not enabled
    if (_backend == nullptr || _inertial_history == nullptr) {
        return;
    }

    _update_swoop_filt = false;

    // append current velocity and attitude correction into history buffer
    struct inertial_data_frame_s inertial_data_newest;
    const AP_AHRS_NavEKF &_ahrs = AP::ahrs_navekf();
    _ahrs.getCorrectedDeltaVelocityNED(inertial_data_newest.correctedVehicleDeltaVelocityNED, inertial_data_newest.dt);
    inertial_data_newest.Tbn = _ahrs.get_rotation_body_to_ned();
    Vector3f curr_vel;
    nav_filter_status status;
    if (!_ahrs.get_velocity_NED(curr_vel) || !_ahrs.get_filter_status(status)) {
        inertial_data_newest.inertialNavVelocityValid = false;
    } else {
        inertial_data_newest.inertialNavVelocityValid = status.flags.horiz_vel;
    }
    curr_vel.z = -curr_vel.z;  // NED to NEU
    inertial_data_newest.inertialNavVelocity = curr_vel;

    inertial_data_newest.time_usec = AP_HAL::micros64();
    _inertial_history->push_force(inertial_data_newest);

    // update estimator of target position
    if (_backend != nullptr && _enabled) {
        _backend->update();
        run_estimator(rangefinder_alt_cm*0.01f, rangefinder_alt_valid);
    }
}

bool AC_PrecLand::target_acquired()
{
    _target_acquired = _target_acquired && (AP_HAL::millis()-_last_update_ms) < (uint8_t)_tacq_timeout*1000.0f;
    return _target_acquired;
}

bool AC_PrecLand::has_been_confident()
{
    return _swoop_has_been_confident;
}

bool AC_PrecLand::get_target_position_cm(Vector2f& ret)
{
    // if (!target_acquired()) {
    //     return false;
    // }
    switch (_estimator_type) {
        case ESTIMATOR_TYPE_RAW_SENSOR: {
            FALLTHROUGH;
        }
        case ESTIMATOR_TYPE_KALMAN_FILTER: {
            
            Vector2f curr_pos;
            if (!AP::ahrs().get_relative_position_NE_origin(curr_pos)) {
                return false;
            }
            ret.x = (_target_pos_rel_out_NE.x + curr_pos.x) * 100.0f;   // m to cm
            ret.y = (_target_pos_rel_out_NE.y  + curr_pos.y) * 100.0f;  // m to cm
            return true;
        }
        case ESTIMATOR_TYPE_SWOOP_FILTER: {
            ret.x = _ave_target_pos_abs_out_NE.x * 100.0f;
            ret.y = _ave_target_pos_abs_out_NE.y * 100.0f;
            return true;
        }
        default: {
            return false;
        }
    }
}

// To be used with SWOOP_FILTER enabled
void AC_PrecLand::update_swoop_target_position_cm()
{
    if (!target_acquired()) {
        return;
    }

    Vector2f curr_pos;
    if (!AP::ahrs().get_relative_position_NE_origin(curr_pos)) {
        return;
    }

    _swoop_filter_confident = false;

    // Newest target absolute estimate
    Vector2f curr_target_abs_NE;

    curr_target_abs_NE.x = (_target_pos_rel_out_NE.x + curr_pos.x);
    curr_target_abs_NE.y = (_target_pos_rel_out_NE.y + curr_pos.y);

    // Add newest target estimate to buffer, replacing oldest sample
    _target_history->push_force(curr_target_abs_NE);

    // If you cull more that CULL_PCNT_MAX outliers and max error is still greater that MAX_ERR
    // it is not safe to land

    // copy of buffer into a new one that we can cull, we don't remember which outliers we culled last loop
    ObjectArray<Vector2f> *_tmp_target_history = new ObjectArray<Vector2f>(_num_ave_samples);
    for (uint8_t k=0; k<_target_history->available(); k++) {
        const Vector2f *sample = (*_target_history)[k];
        _tmp_target_history->push_force(*sample);
    }

    int max_cull_samples = (int)_num_ave_samples*_max_cull_pct;
    Vector2f max_err = Vector2f(0.0f, 0.0f);
    Vector2f err = Vector2f(0.0f, 0.0f);
    uint16_t err_index = 0;
    uint16_t cull_counter = 0;

    // if buffer not full, don't bother processing
    if (_target_history->space()==0){

        for (int l=0; l<max_cull_samples; l++){

            // get average
            _ave_target_pos_abs_out_NE = get_buffer_average(_tmp_target_history);

            // get max error
            err = Vector2f(0.0f, 0.0f);
            max_err = Vector2f(0.0f, 0.0f);
            float err_length;
            for (uint8_t j=0; j<_tmp_target_history->available(); j++) {
                const Vector2f *_err_target_data = (*_tmp_target_history)[j];
                err.x = abs(_ave_target_pos_abs_out_NE.x - _err_target_data->x);
                err.y = abs(_ave_target_pos_abs_out_NE.y - _err_target_data->y);
                err_length = err.length();

                // update largest error
                if (err_length > abs(max_err.length())){
                    err_index = j;
                    max_err.x = err.x;
                    max_err.y = err.y;
                }
            }

            // break if acceptable, cull largest error and repeat if not acceptable
            if (max_err.length()>_acceptable_target_error_cm*0.01f){
                _tmp_target_history->remove(err_index);
                cull_counter += 1;
            }else{
                _swoop_filter_confident = true;
                _swoop_has_been_confident = true;
                break;
            }
        }

    AP::logger().Write("PLFT", "TimeUS,c,o,valid,cX,cY,aX,aY,e", "QIIIfffff",
                                        AP_HAL::micros64(),
                                        (uint32_t)_target_history->available(),
                                        (uint32_t)cull_counter,
                                        (uint16_t)_swoop_filter_confident,
                                        (float)curr_target_abs_NE.x,
                                        (float)curr_target_abs_NE.y,
                                        (float)_ave_target_pos_abs_out_NE.x,
                                        (float)_ave_target_pos_abs_out_NE.y,
                                        (float)max_err.length());
    }
}

Vector2f AC_PrecLand::get_buffer_average(ObjectArray<Vector2f> *buffer){

    Vector2f sum = Vector2f(0.0f, 0.0f);
    Vector2f ave;
    for (uint8_t i=0; i<buffer->available(); i++) {
        const Vector2f *sample = (*buffer)[i];
        sum.x += sample->x;
        sum.y += sample->y;
    }
    ave.x = sum.x/(buffer->available());
    ave.y = sum.y/(buffer->available());

    return ave;
}

void AC_PrecLand::get_target_position_measurement_cm(Vector3f& ret)
{
    ret = _target_pos_rel_meas_NED*100.0f;
    return;
}

bool AC_PrecLand::get_target_position_relative_cm(Vector2f& ret)
{
    if (!target_acquired()) {
        return false;
    }
    ret = _target_pos_rel_out_NE*100.0f;
    return true;
}

bool AC_PrecLand::get_target_velocity_relative_cms(Vector2f& ret)
{
    // Swoop Filter assumes zero velocity
    if (!target_acquired() || _estimator_type==ESTIMATOR_TYPE_SWOOP_FILTER) {
        return false;
    }
    ret = _target_vel_rel_out_NE*100.0f;
    return true;
}

float AC_PrecLand::get_target_distance_scalar(void)
{
    return norm(_target_pos_rel_out_NE.x, _target_pos_rel_out_NE.y)*100.0f;
}

uint32_t AC_PrecLand::get_lag(void)
{
    return _backend->get_lag();
}

// handle_msg - Process a LANDING_TARGET mavlink message
void AC_PrecLand::handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
    // run backend update
    if (_backend != nullptr) {
        _backend->handle_msg(packet, timestamp_ms);
    }
}

//
// Private methods
//

void AC_PrecLand::run_estimator(float rangefinder_alt_m, bool rangefinder_alt_valid)
{
    const struct inertial_data_frame_s *inertial_data_delayed = (*_inertial_history)[0];

    switch (_estimator_type) {
        case ESTIMATOR_TYPE_SWOOP_FILTER: {
            FALLTHROUGH;
        }
        case ESTIMATOR_TYPE_RAW_SENSOR: {
            // Return if there's any invalid velocity data
            for (uint8_t i=0; i<_inertial_history->available(); i++) {
                const struct inertial_data_frame_s *inertial_data = (*_inertial_history)[i];
                if (!inertial_data->inertialNavVelocityValid) {
                    _target_acquired = false;
                    return;
                }
            }

            // Predict
            if (target_acquired()) {
                _target_pos_rel_est_NE.x -= inertial_data_delayed->inertialNavVelocity.x * inertial_data_delayed->dt;
                _target_pos_rel_est_NE.y -= inertial_data_delayed->inertialNavVelocity.y * inertial_data_delayed->dt;
                _target_vel_rel_est_NE.x = -inertial_data_delayed->inertialNavVelocity.x;
                _target_vel_rel_est_NE.y = -inertial_data_delayed->inertialNavVelocity.y;
            }

            // Update if a new Line-Of-Sight measurement is available
            if (construct_pos_meas_using_rangefinder(rangefinder_alt_m, rangefinder_alt_valid)) {
                _target_pos_rel_est_NE.x = _target_pos_rel_meas_NED.x;
                _target_pos_rel_est_NE.y = _target_pos_rel_meas_NED.y;
                _target_vel_rel_est_NE.x = -inertial_data_delayed->inertialNavVelocity.x;
                _target_vel_rel_est_NE.y = -inertial_data_delayed->inertialNavVelocity.y;

                _last_update_ms = AP_HAL::millis();
                _target_acquired = true;
                _update_swoop_filt = true;
            }

            // Output prediction
            if (target_acquired()) {
                run_output_prediction();
                if(_estimator_type==ESTIMATOR_TYPE_SWOOP_FILTER && _update_swoop_filt){
                    update_swoop_target_position_cm();
                }
            }
            break;
        }
        case ESTIMATOR_TYPE_KALMAN_FILTER: {
            // Predict
            if (target_acquired()) {
                const float& dt = inertial_data_delayed->dt;
                const Vector3f& vehicleDelVel = inertial_data_delayed->correctedVehicleDeltaVelocityNED;

                _ekf_x.predict(dt, -vehicleDelVel.x, _accel_noise*dt);
                _ekf_y.predict(dt, -vehicleDelVel.y, _accel_noise*dt);
            }

            // Update if a new Line-Of-Sight measurement is available
            if (construct_pos_meas_using_rangefinder(rangefinder_alt_m, rangefinder_alt_valid)) {
                float xy_pos_var = sq(_target_pos_rel_meas_NED.z*(0.01f + 0.01f*AP::ahrs().get_gyro().length()) + 0.02f);
                if (!target_acquired()) {
                    // reset filter state
                    if (inertial_data_delayed->inertialNavVelocityValid) {
                        _ekf_x.init(_target_pos_rel_meas_NED.x, xy_pos_var, -inertial_data_delayed->inertialNavVelocity.x, sq(2.0f));
                        _ekf_y.init(_target_pos_rel_meas_NED.y, xy_pos_var, -inertial_data_delayed->inertialNavVelocity.y, sq(2.0f));
                    } else {
                        _ekf_x.init(_target_pos_rel_meas_NED.x, xy_pos_var, 0.0f, sq(10.0f));
                        _ekf_y.init(_target_pos_rel_meas_NED.y, xy_pos_var, 0.0f, sq(10.0f));
                    }
                    _last_update_ms = AP_HAL::millis();
                    _target_acquired = true;
                } else {
                    float NIS_x = _ekf_x.getPosNIS(_target_pos_rel_meas_NED.x, xy_pos_var);
                    float NIS_y = _ekf_y.getPosNIS(_target_pos_rel_meas_NED.y, xy_pos_var);
                    if (MAX(NIS_x, NIS_y) < 3.0f || _outlier_reject_count >= 3) {
                        _outlier_reject_count = 0;
                        _ekf_x.fusePos(_target_pos_rel_meas_NED.x, xy_pos_var);
                        _ekf_y.fusePos(_target_pos_rel_meas_NED.y, xy_pos_var);
                        _last_update_ms = AP_HAL::millis();
                        _target_acquired = true;
                    } else {
                        _outlier_reject_count++;
                    }
                }
            }

            // Output prediction
            if (target_acquired()) {
                _target_pos_rel_est_NE.x = _ekf_x.getPos();
                _target_pos_rel_est_NE.y = _ekf_y.getPos();
                _target_vel_rel_est_NE.x = _ekf_x.getVel();
                _target_vel_rel_est_NE.y = _ekf_y.getVel();

                run_output_prediction();
            }
            break;
        }
    }
}

bool AC_PrecLand::target_pos_confident()
{
    //TODO implement a measure of the swoop filter's confidence
    switch (_estimator_type) {
        case ESTIMATOR_TYPE_SWOOP_FILTER:
        {
            return _swoop_filter_confident;
        }
        case ESTIMATOR_TYPE_RAW_SENSOR: 
        {
            return true;
        }
        case ESTIMATOR_TYPE_KALMAN_FILTER: {
            return true;
        }
    }    
    return false;
}

bool AC_PrecLand::retrieve_los_meas(Vector3f& target_vec_unit_body)
{
    Vector3f raw_target_vec_unit_body;
    if (_backend->have_los_meas() && _backend->los_meas_time_ms() != _last_backend_los_meas_ms) {
        _last_backend_los_meas_ms = _backend->los_meas_time_ms();
        _backend->get_los_body(raw_target_vec_unit_body);

        // Apply sensor alignment rotation
        Matrix3f sensor_rot;
        sensor_rot.from_euler(radians(-_cam_offset_ang.get().x*0.01f), radians(-_cam_offset_ang.get().y*0.01f), radians(-_cam_offset_ang.get().z*0.01f));
        target_vec_unit_body = sensor_rot*raw_target_vec_unit_body;

        return true;
    } else {
        return false;
    }
}

bool AC_PrecLand::construct_pos_meas_using_rangefinder(float rangefinder_alt_m, bool rangefinder_alt_valid)
{
    Vector3f target_vec_unit_body;
    if (retrieve_los_meas(target_vec_unit_body)) {
        const struct inertial_data_frame_s *inertial_data_delayed = (*_inertial_history)[0];

        Vector3f target_vec_unit_ned = inertial_data_delayed->Tbn * target_vec_unit_body;
        bool target_vec_valid = target_vec_unit_ned.z > 0.0f;
        bool alt_valid = (rangefinder_alt_valid && rangefinder_alt_m > 0.0f) || (_backend->distance_to_target() > 0.0f);
        if (target_vec_valid && alt_valid) {
            float dist, alt;
            if (_backend->distance_to_target() > 0.0f) {
                dist = _backend->distance_to_target();
                alt = dist * target_vec_unit_ned.z;
            } else {
                alt = MAX(rangefinder_alt_m, 0.0f);
                dist = alt / target_vec_unit_ned.z;
            }

            // Compute camera position relative to IMU
            Vector3f accel_body_offset = AP::ins().get_imu_pos_offset(AP::ahrs().get_primary_accel_index());
            Vector3f cam_pos_ned = inertial_data_delayed->Tbn * (_cam_offset.get() - accel_body_offset);

            // Compute target position relative to IMU
            _target_pos_rel_meas_NED = Vector3f(target_vec_unit_ned.x*dist, target_vec_unit_ned.y*dist, alt) + cam_pos_ned;

         
            
            AP::logger().Write("PLCL", "TimeUS,ts,bX,bY,bZ,nX,nY,nZ,tX,tY,tZ,tL", "QIffffffffff",
                                        AP_HAL::micros64(),
                                        (uint32_t)inertial_data_delayed->time_usec,
                                        (float)target_vec_unit_body.x,
                                        (float)target_vec_unit_body.y,
                                        (float)target_vec_unit_body.z,
                                        (float)target_vec_unit_ned.x,
                                        (float)target_vec_unit_ned.y,
                                        (float)target_vec_unit_ned.z,
                                        (float)_target_pos_rel_meas_NED.x,
                                        (float)_target_pos_rel_meas_NED.y,
                                        (float)_target_pos_rel_meas_NED.z,
                                        (float)_target_pos_rel_meas_NED.length());

            // Test if target is within a reasonable distance to craft
            // (prevents unrealistically large estimated distances which probably aren't real)
            Vector2f target_pos_rel_lateral_NE;
            target_pos_rel_lateral_NE = Vector2f(_target_pos_rel_meas_NED.x, _target_pos_rel_meas_NED.y);
            if(target_pos_rel_lateral_NE.length()> _max_target_distance) {
                return false;
            }                    
            return true;
        }
    }
    return false;
}

void AC_PrecLand::run_output_prediction()
{
    _target_pos_rel_out_NE = _target_pos_rel_est_NE;
    _target_vel_rel_out_NE = _target_vel_rel_est_NE;

    // Predict forward from delayed time horizon
    for (uint8_t i=1; i<_inertial_history->available(); i++) {
        const struct inertial_data_frame_s *inertial_data = (*_inertial_history)[i];
        _target_vel_rel_out_NE.x -= inertial_data->correctedVehicleDeltaVelocityNED.x;
        _target_vel_rel_out_NE.y -= inertial_data->correctedVehicleDeltaVelocityNED.y;
        _target_pos_rel_out_NE.x += _target_vel_rel_out_NE.x * inertial_data->dt;
        _target_pos_rel_out_NE.y += _target_vel_rel_out_NE.y * inertial_data->dt;
    }

    const AP_AHRS &_ahrs = AP::ahrs();

    const Matrix3f& Tbn = (*_inertial_history)[_inertial_history->available()-1]->Tbn;
    Vector3f accel_body_offset = AP::ins().get_imu_pos_offset(_ahrs.get_primary_accel_index());

    // Apply position correction for CG offset from IMU
    Vector3f imu_pos_ned = Tbn * accel_body_offset;
    _target_pos_rel_out_NE.x += imu_pos_ned.x;
    _target_pos_rel_out_NE.y += imu_pos_ned.y;

    // Apply position correction for body-frame horizontal camera offset from CG, so that vehicle lands lens-to-target
    Vector3f cam_pos_horizontal_ned = Tbn * Vector3f(_cam_offset.get().x, _cam_offset.get().y, 0);
    _target_pos_rel_out_NE.x -= cam_pos_horizontal_ned.x;
    _target_pos_rel_out_NE.y -= cam_pos_horizontal_ned.y;

    // Apply velocity correction for IMU offset from CG
    Vector3f vel_ned_rel_imu = Tbn * (_ahrs.get_gyro() % (-accel_body_offset));
    _target_vel_rel_out_NE.x -= vel_ned_rel_imu.x;
    _target_vel_rel_out_NE.y -= vel_ned_rel_imu.y;

    // Apply land offset
    Vector3f land_ofs_ned_m = _ahrs.get_rotation_body_to_ned() * Vector3f(_land_ofs_cm_x,_land_ofs_cm_y,0) * 0.01f;
    _target_pos_rel_out_NE.x += land_ofs_ned_m.x;
    _target_pos_rel_out_NE.y += land_ofs_ned_m.y;

    AP::logger().Write("PLKF", "TimeUS,tsmp,vX,vY,pX,pY", "QIffff",
                                        AP_HAL::micros64(),
                                        (uint32_t)(*_inertial_history)[_inertial_history->available()-1]->time_usec,
                                        (float)_target_vel_rel_out_NE.x,
                                        (float)_target_vel_rel_out_NE.y,
                                        (float)_target_pos_rel_out_NE.x,
                                        (float)_target_pos_rel_out_NE.y);
}

bool AC_PrecLand::timeout(void){

    // if timer has been started and since expired
    if ((_commence_time == 0) || (AP_HAL::millis() - _commence_time) >= (uint32_t)(_timeout.get()*1000)){
        return true;
    }else{
        return false;
    }
}

bool AC_PrecLand::backed_initialised(void){
    if (_backend != nullptr){
        return true;
    }
    return false;
}

void AC_PrecLand::set_enabled(bool enabled){
    _enabled = enabled;
}

bool AC_PrecLand::can_abort(float hagl){
    if(hagl > _min_abort_alt){
        return true;
    }
    return false;
}