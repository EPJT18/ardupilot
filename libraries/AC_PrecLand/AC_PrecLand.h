#pragma once

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdint.h>
#include "PosVelEKF.h"
#include <AP_HAL/utility/RingBuffer.h>

// declare backend classes
class AC_PrecLand_Backend;
class AC_PrecLand_Companion;
class AC_PrecLand_IRLock;
class AC_PrecLand_SITL_Gazebo;
class AC_PrecLand_SITL;

class AC_PrecLand
{
    // declare backends as friends
    friend class AC_PrecLand_Backend;
    friend class AC_PrecLand_Companion;
    friend class AC_PrecLand_IRLock;
    friend class AC_PrecLand_SITL_Gazebo;
    friend class AC_PrecLand_SITL;

public:
    AC_PrecLand();

    /* Do not allow copies */
    AC_PrecLand(const AC_PrecLand &other) = delete;
    AC_PrecLand &operator=(const AC_PrecLand&) = delete;

    // precision landing behaviours (held in PRECLAND_ENABLED parameter)
    enum PrecLandBehaviour {
        PRECLAND_BEHAVIOUR_DISABLED,
        PRECLAND_BEHAVIOR_ALWAYSLAND,
        PRECLAND_BEHAVIOR_CAUTIOUS
    };

    // types of precision landing (used for PRECLAND_TYPE parameter)
    enum PrecLandType {
        PRECLAND_TYPE_NONE = 0,
        PRECLAND_TYPE_COMPANION,
        PRECLAND_TYPE_IRLOCK,
        PRECLAND_TYPE_SITL_GAZEBO,
        PRECLAND_TYPE_SITL,
    };

    // perform any required initialisation of landing controllers
    // update_rate_hz should be the rate at which the update method will be called in hz
    void init(uint16_t update_rate_hz);

    // reinit precision landing immediately before use
    void reinit(void);

    // returns true if precision landing is healthy
    bool healthy() const { return _backend_state.healthy; }

    // returns true if precision landing is enabled
    bool enabled() const { return _enabled.get(); }

    // returns true if behaviour set to abort if target pos is deemed not confident
    bool abort_if_not_confident();

    // returns time of last update
    uint32_t last_update_ms() const { return _last_update_ms; }

    // returns time of last time target was seen
    uint32_t last_backend_los_meas_ms() const { return _last_backend_los_meas_ms; }

    // returns estimator type
    uint8_t estimator_type() const { return _estimator_type; }

    // returns ekf outlier count
    uint32_t ekf_outlier_count() const { return _outlier_reject_count; }

    // give chance to driver to get updates from sensor, should be called at 400hz
    void update(float rangefinder_alt_cm, bool rangefinder_alt_valid);

    // returns target position relative to the EKF origin
    bool get_target_position_cm(Vector2f& ret);

    // returns target relative position as 3D vector
    void get_target_position_measurement_cm(Vector3f& ret);

    // returns target position relative to vehicle
    bool get_target_position_relative_cm(Vector2f& ret);

    // returns target position relative to vehicle
    bool get_raw_target_position_relative_cm(Vector2f& ret);

    // returns target velocity relative to vehicle
    bool get_target_velocity_relative_cms(Vector2f& ret);

    // returns target location as a GPS coordinate, pass in vehicle loc
    Location get_target_est_loc(void);

    // returns target location as a GPS coordinate, pass in vehicle loc
    Location get_raw_target_est_loc(void);

    // returns scalar distance from vehicle to target (cm)
    float get_target_distance_scalar(void);

    // returns the covariance of the target x pos kalman filter
    float get_ekf_x_cov(void);

    // returns the covariance of the target y pos kalman filter
    float get_ekf_y_cov(void);

    // returns true when the landing target has been detected
    bool target_acquired();

    // get the lag of the most recent packet
    uint32_t get_lag();

    // process a LANDING_TARGET mavlink message
    void handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms);

    // returns true when we are confident of the target's position
    bool target_pos_confident(void);

    // set precland behaviour online
    void set_online_behaviour(uint8_t enabled_status){ _enabled_online = enabled_status; }
    
    // get precland behaviour online
    enum PrecLandBehaviour get_online_behaviour() { return (enum PrecLandBehaviour)(_enabled_online); }

    // landing timeout
    bool timeout(void);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:
    enum estimator_type_t {
        ESTIMATOR_TYPE_RAW_SENSOR = 0,
        ESTIMATOR_TYPE_KALMAN_FILTER = 1
    };

    // returns enabled parameter as an behaviour
    enum PrecLandBehaviour get_behaviour() const { return (enum PrecLandBehaviour)(_enabled.get()); }

    // run target position estimator
    void run_estimator(float rangefinder_alt_m, bool rangefinder_alt_valid);

    // If a new measurement was retrieved, sets _target_pos_rel_meas_NED and returns true
    bool construct_pos_meas_using_rangefinder(float rangefinder_alt_m, bool rangefinder_alt_valid);

    // get vehicle body frame 3D vector from vehicle to target.  returns true on success, false on failure
    bool retrieve_los_meas(Vector3f& target_vec_unit_body);

    // calculate target's position and velocity relative to the vehicle (used as input to position controller)
    // results are stored in_target_pos_rel_out_NE, _target_vel_rel_out_NE
    void run_output_prediction(Vector2f& target_pos_rel_out_NE, Vector2f& target_vel_rel_out_NE, Vector2f target_pos_rel_est_NE, Vector2f target_vel_rel_est_NE);

    // parameters
    AP_Int8                     _enabled;           // enabled/disabled and behaviour
    AP_Int8                     _type;              // precision landing sensor type
    AP_Int8                     _bus;               // which sensor bus
    AP_Int8                     _estimator_type;    // precision landing estimator type
    AP_Float                    _lag;               // sensor lag in seconds
    AP_Float                    _yaw_align;         // Yaw angle from body x-axis to sensor x-axis.
    AP_Float                    _land_ofs_cm_x;     // Desired landing position of the camera forward of the target in vehicle body frame
    AP_Float                    _land_ofs_cm_y;     // Desired landing position of the camera right of the target in vehicle body frame
    AP_Float                    _accel_noise;       // accelerometer process noise
    AP_Vector3f                 _cam_offset;        // Position of the camera relative to the CG
    AP_Int8                     _timeout;           // Target search timeout (plane abort)
    AP_Int8                     _tacq_timeout;      // Target acquired timeout (reset ekf)

    uint8_t                     _enabled_online;    // behaviour set by mission (online), does not write to param EEPROM
    uint32_t                    _last_update_ms;    // system time in millisecond when update was last called
    uint32_t                    _commence_time;     // Timestamp when precision landing commences looking for the target, used in timeout
    bool                        _target_acquired;   // true if target has been seen recently
    uint32_t                    _last_backend_los_meas_ms;  // system time target was last seen

    PosVelEKF                   _ekf_x, _ekf_y;     // Kalman Filter for x and y axis
    uint32_t                    _outlier_reject_count;  // mini-EKF's outlier counter (3 consecutive outliers lead to EKF accepting updates)
    AP_Float                    _covariance_threshold; // EKF covariance confidence threshold

    Vector3f                    _target_pos_rel_meas_NED; // target's relative position as 3D vector

    Vector2f                    _target_pos_rel_est_NE; // target's position relative to the IMU, not compensated for lag
    Vector2f                    _target_vel_rel_est_NE; // target's velocity relative to the IMU, not compensated for lag

    Vector2f                    _target_pos_rel_out_NE; // target's position relative to the camera, fed into position controller
    Vector2f                    _target_vel_rel_out_NE; // target's velocity relative to the CG, fed into position controller

    // logging of raw est alongside EKF, this isn't used if PLND_TYPE=0
    Vector3f                    _raw_target_pos_rel_meas_NED; // target's relative position as 3D vector
    Vector2f                    _raw_target_pos_rel_est_NE; // target's position relative to the IMU, not compensated for lag
    Vector2f                    _raw_target_vel_rel_est_NE; // target's velocity relative to the IMU, not compensated for lag
    Vector2f                    _raw_target_pos_rel_out_NE; // raw target's position relative to the camera, fed into position controller
    Vector2f                    _raw_target_vel_rel_out_NE; // raw target's velocity relative to the CG, fed into position controller

    // structure and buffer to hold a history of vehicle velocity
    struct inertial_data_frame_s {
        Matrix3f Tbn;                               // dcm rotation matrix to rotate body frame to north
        Vector3f correctedVehicleDeltaVelocityNED;
        Vector3f inertialNavVelocity;
        bool inertialNavVelocityValid;
        float dt;
        uint64_t time_usec;
    };
    ObjectArray<inertial_data_frame_s> *_inertial_history;

    // backend state
    struct precland_state {
        bool    healthy;
    } _backend_state;
    AC_PrecLand_Backend         *_backend;  // pointers to backend precision landing driver
};
