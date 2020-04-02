#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Swoop.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_Swoop::AP_BattMonitor_Swoop (AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Analog(mon, mon_state, params)
{
    init_ms = AP_HAL::millis();
    battery_state_initialised = false;
}

void AP_BattMonitor_Swoop::init(void) {
    switch (_params._chemistry.get()){
        case BATTERY_TYPE_LGMJ1:
            _zero_voltage = LGMJ1_zero_voltage;
            _battery_lookup_table = LGMJ1;
            _table_size = ARRAY_SIZE(LGMJ1);
            break;
        case BATTERY_TYPE_TATTU:
            _zero_voltage = tattu_zero_voltage;
            _battery_lookup_table = tattu;
            _table_size = ARRAY_SIZE(tattu);
            break;
    }
    _dead_cell_state = lookup_table_from_volt(_zero_voltage);
}

// read - read the voltage and current. Update estimated states
void AP_BattMonitor_Swoop::read()
{
    AP_BattMonitor_Analog::read();

    check_health();

    // updated estimates based on lookup table
    if (lookup_valid()){
        struct soc_lookup_t cell_state = lookup_table_from_volt(_state.voltage_resting_estimate/_params._series.get());
        _state.consumed_mah_lookup = 1000.0f * cell_state.ah_used * _params._parallel.get();
        _state.consumed_wh_lookup = cell_state.wh_used * _params._parallel.get() * _params._series.get();

        // initialise wh remaining on boot if haven't already done so
        if (!battery_state_initialised){
            
            _initial_cell_state = cell_state;
            _initial_wh_remaining = (_dead_cell_state.wh_used - _initial_cell_state.wh_used) * _params._parallel.get() * _params._series.get();
            battery_state_initialised = true;
        }
        AP_Logger *logger = AP_Logger::get_singleton();
    logger->Write("BH2", "TimeUS,sense,lookup", "Qff",
                                        AP_HAL::micros64(),
                                        (float)_initial_wh_remaining - _state.consumed_wh,
                                        (float)(_dead_cell_state.wh_used*_params._parallel.get()*_params._series.get()) - _state.consumed_wh_lookup);
    }

    // if the battery monitor is healthy, use current-sensed wh consumed estimate
    // else revert to lookup table
    if (!_state.healthy && lookup_valid()){
        _state.remaining_wh = (_dead_cell_state.wh_used*_params._parallel.get()*_params._series.get()) - _state.consumed_wh_lookup;
    }else{
        _state.remaining_wh = _initial_wh_remaining - _state.consumed_wh;
    }

    
}

bool AP_BattMonitor_Swoop::check_health(void){

    // if lookup hasn't been initialised yet, nothing to check against
    // scope to change later
    if (!lookup_valid()){
        _state.healthy = true;
        return true;
    }

    // back-calculate estimated resting voltage based on sensed wh and lookup wh
    float v_est_sensed = lookup_table_from_wh(_initial_cell_state.wh_used + _state.consumed_wh/(_params._series.get()*_params._parallel.get())).voltage;
    float v_est_lookup = lookup_table_from_wh(_state.consumed_wh_lookup/(_params._series.get()*_params._parallel.get())).voltage;

    AP_Logger *logger = AP_Logger::get_singleton();
    logger->Write("BHLT", "TimeUS,vs, vl", "Qff",
                                        AP_HAL::micros64(),
                                        (float)v_est_sensed,
                                        (float)v_est_lookup);

    if (abs(v_est_sensed - v_est_lookup) < _params._allowable_v_est_deviation.get()){
        _state.healthy = true;
        return true;
    }

    _state.healthy = false;
    return false;
}

bool AP_BattMonitor_Swoop::lookup_valid(void){
    uint32_t now_ms = AP_HAL::millis();

    // wait 10sec to settle
    if ((now_ms - init_ms) > 10000){
        return true;
    }
    return false;
}

struct soc_lookup_t AP_BattMonitor_Swoop::lookup_table_from_volt(float voltage)
{

    float per_cell = voltage;
    struct soc_lookup_t current_state = {per_cell, 0, 0};

    if (per_cell >= _battery_lookup_table[0].voltage) {
        return current_state;
    }

    for (uint8_t i=1; i<_table_size; i++) {
        if (per_cell >= _battery_lookup_table[i].voltage) {

            // interpolate voltage
            float dv1 = per_cell - _battery_lookup_table[i].voltage;
            float dv2 = _battery_lookup_table[i-1].voltage - _battery_lookup_table[i].voltage;

            float ah1 = _battery_lookup_table[i].ah_used;
            float ah2 = _battery_lookup_table[i-1].ah_used;

            float wh1 = _battery_lookup_table[i].wh_used;
            float wh2 = _battery_lookup_table[i-1].wh_used;

            float ah = ah1+(dv1/dv2)*(ah2-ah1);
            float wh = wh1+(dv1/dv2)*(wh2-wh1);

            current_state = {voltage, ah, wh};
            return current_state;
        }
    }
    // off the bottom of the table
    current_state = {
        _battery_lookup_table[_table_size-1].voltage,
        _battery_lookup_table[_table_size-1].ah_used,
        _battery_lookup_table[_table_size-1].wh_used};
    return current_state;
}


struct soc_lookup_t AP_BattMonitor_Swoop::lookup_table_from_wh(float wh)
{

    float per_cell = wh;
    struct soc_lookup_t current_state = {per_cell, 0, 0};

    if (per_cell <= _battery_lookup_table[0].wh_used) {
        current_state = {_battery_lookup_table[0].voltage, 0, 0};
        return current_state;
    }

    for (uint8_t i=1; i<_table_size; i++) {
        if (per_cell <= _battery_lookup_table[i].wh_used) {

            // interpolate wh
            float dv1 = _battery_lookup_table[i].voltage;
            float dv2 = _battery_lookup_table[i-1].voltage;

            float ah1 = _battery_lookup_table[i].ah_used;
            float ah2 = _battery_lookup_table[i-1].ah_used;

            float wh1 = per_cell - _battery_lookup_table[i].wh_used;
            float wh2 = _battery_lookup_table[i-1].wh_used - _battery_lookup_table[i].wh_used;

            float dv = dv1+(wh1/wh2)*(dv2-dv1);
            float ah = ah1+(wh1/wh2)*(ah2-ah1);

            current_state = {dv, ah, per_cell};
            return current_state;
        }
    }
    // off the bottom of the table
    current_state = {
        _battery_lookup_table[_table_size-1].voltage,
        _battery_lookup_table[_table_size-1].ah_used,
        _battery_lookup_table[_table_size-1].wh_used};
    return current_state;
}

struct soc_lookup_t AP_BattMonitor_Swoop::lookup_table_from_ah(float ah)
{

    float per_cell = ah;
    struct soc_lookup_t current_state = {per_cell, 0, 0};

    if (per_cell <= _battery_lookup_table[0].ah_used) {
        current_state = {_battery_lookup_table[0].voltage, 0, 0};
        return current_state;
    }

    for (uint8_t i=1; i<_table_size; i++) {
        if (per_cell <= _battery_lookup_table[i].ah_used) {

            // interpolate ah
            float dv1 = _battery_lookup_table[i].voltage;
            float dv2 = _battery_lookup_table[i-1].voltage;

            float ah1 = per_cell - _battery_lookup_table[i].ah_used;
            float ah2 = _battery_lookup_table[i-1].ah_used - _battery_lookup_table[i].ah_used;

            float wh1 = _battery_lookup_table[i].wh_used;
            float wh2 = _battery_lookup_table[i-1].wh_used;

            float dv = dv1+(ah1/ah2)*(dv2-dv1);
            float wh = wh1+(ah1/ah2)*(wh2-wh1);

            current_state = {dv, per_cell, wh};
            return current_state;
        }
    }
    // off the bottom of the table
    current_state = {
        _battery_lookup_table[_table_size-1].voltage,
        _battery_lookup_table[_table_size-1].ah_used,
        _battery_lookup_table[_table_size-1].wh_used};
    return current_state;
}