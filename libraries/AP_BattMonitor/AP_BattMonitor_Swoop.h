#pragma once

#include "AP_BattMonitor_Analog.h"
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

/*
  state of charge table for a battery. This should be chosen based on
  a battery that is at maximum age of batteries that will be used in
  the vehicle
 */

struct soc_lookup_t{
    float voltage;
    float ah_used;
    float wh_used;
};

// lipo is 3.5

// voltage, ah used, wh used


const float LGMJ1_zero_voltage = 3.0f;
const soc_lookup_t LGMJ1[] = {
{ 4.2,      0.000023,      0.000097  },
{ 4.15,     0.014561,      0.060472  },
{ 4.1,      0.161815,      0.666884  },
{ 4.05, 	0.472671,      1.933298  },
{ 4,        0.658402,      2.680598  },
{ 3.95, 	0.820916,      3.326155  },
{ 3.9,      0.994431,      4.006662  },
{ 3.85, 	1.153183,      4.621462  },
{ 3.8,      1.304029,      5.197908  },
{ 3.75, 	1.476984,      5.850476  },
{ 3.7,      1.652434,      6.503496  },
{ 3.65, 	1.86204 ,      7.272909  },
{ 3.6,      2.12195 ,      8.214201  },
{ 3.55, 	2.318179,      8.915563  },
{ 3.5,      2.457862,      9.407688  },
{ 3.45, 	2.674712,      10.160627 },
{ 3.4,      2.756245,      10.439949 },
{ 3.35, 	2.823518,      10.666804 },
{ 3.3,      2.88294 ,      10.864302 },
{ 3.25, 	2.933851,      11.030931 },
{ 3.2,      2.979731,      11.178738 },
{ 3.15, 	3.025556,      11.32413  },
{ 3.1,      3.073702,      11.474477 },
{ 3.05, 	3.113833,      11.59781  },
{ 3,        3.147387,      11.699245 },
{ 2.95, 	3.173478,      11.776817 },
{ 2.9,      3.194097,      11.837096 },
{ 2.85, 	3.211069,      11.885862 },
{ 2.8,      3.224057,      11.922524 }};

const float tattu_zero_voltage = 3.5f;
const soc_lookup_t tattu[] = {
{ 4.35,		0,	           0         },
{ 4.3,		0.0619,        0.2677175 },
{ 4.25,		0.1376,        0.591335  },
{ 4.2,		0.2132,        0.910745  },
{ 4.15,		0.2957,        1.2551825 },
{ 4.1,		0.3783,        1.5959075 },
{ 4.05,		0.4677,        1.9602125 },
{ 4,		0.5433,        2.2645025 },
{ 3.95,		0.6327,        2.6198675 },
{ 3.9,		0.7359,        3.0249275 },
{ 3.85,		0.8734,        3.55774   },
{ 3.8,		1.0591,        4.2680425 },
{ 3.75,		1.3136,        5.22878   },
{ 3.7,		1.513,         5.971545 },
{ 3.65,		1.6712,        6.55293   },
{ 3.6,		1.6987,        6.6526175 },
{ 3.55,		1.7193,        6.7262625 },
{ 3.5,		1.7331,        6.7749075 },
{ 3.45,		1.7469,        6.8228625 },
{ 3.4,		1.7537,        6.8461525 },
{ 3.35,		1.7606,        6.86944   },
{ 3.3,		1.7675,        6.8923825 },
{ 3.25,		1.7744,        6.91498   },
{ 3.2,		1.7813,        6.9372325 },
{ 3.1,		1.7881,        6.9586525 },
{ 3,		1.7911,        6.9678025 }};

enum battery_type{
    BATTERY_TYPE_LGMJ1 = 0,
    BATTERY_TYPE_TATTU,
};

class AP_BattMonitor_Swoop : public AP_BattMonitor_Analog
{
public:

    /// Constructor
    AP_BattMonitor_Swoop(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// consumed_wh_lookup - returns total energy drawn estimate (from lookup table) in watt.hours
    bool consumed_wh_lookup(float&wh, const uint8_t instance = AP_BATT_PRIMARY_INSTANCE) const WARN_IF_UNUSED;

    // value between 0 and 100% capacity remaining from voltage lookup method
    uint8_t capacity_remaining_pct_from_voltage();

    void init(void) override;
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return true; }

    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; };

        /// returns true if battery monitor instance provides consumed current info via a lookup table
    bool has_consumed_current_lookup() const override { return true; }

    /// returns true if battery monitor instance provides consumed energy info via a lookup table
    bool has_consumed_energy_lookup() const override { return true; }

    // returns true if battery monitor provides individual cell voltages
    bool has_wh_remaining() const override { return true; }

private:

    // Return average cell lookup state from voltage input
    struct soc_lookup_t lookup_table_from_volt(float voltage);

    // Return average cell lookup state from ah input
    struct soc_lookup_t lookup_table_from_ah(float ah);

    // Return average cell lookup state from wh input
    struct soc_lookup_t lookup_table_from_wh(float wh);

    // Calculate estimated Wh remaining from current sensor, if health is bad, revert to lookup table
    bool check_health(void);

    // Lookup table is only valid if we have an accurate resting voltage estimate
    bool lookup_valid(void);


protected:

    AP_HAL::AnalogSource *_volt_pin_analog_source;
    AP_HAL::AnalogSource *_curr_pin_analog_source;
    const soc_lookup_t *_battery_lookup_table;
    float _zero_voltage;
    soc_lookup_t _initial_cell_state;
    soc_lookup_t _dead_cell_state;
    uint8_t _table_size;
    float _cell_capacity;
    bool battery_state_initialised;
    float _initial_wh_remaining;
    uint32_t init_ms;
};
