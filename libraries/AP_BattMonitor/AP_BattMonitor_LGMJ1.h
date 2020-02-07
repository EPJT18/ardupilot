#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"
#include "AP_BattMonitor_LGMJ1.h"

class AP_BattMonitor_LGMJ1 : public AP_BattMonitor_Analog
{
    public:
    AP_BattMonitor_LGMJ1(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params) : AP_BattMonitor_Analog(mon, mon_state, params) {}
    
    void read() override;

    bool has_current() const override {return true;}
    
    private:
    uint32_t last_armed_ms;
    bool using_table = true;

    // lookup SoC in table, returning SoC as a percentage
    float lookup_SoC_table(float voltage);


}