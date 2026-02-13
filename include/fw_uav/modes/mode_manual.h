#pragma once

#include "fw_uav/modes/flight_mode_manager.h"

namespace fw_uav {

// Manual mode: direct RC passthrough to control surfaces
//
// No stabilization or autopilot assistance. RC stick inputs map directly
// to control surface deflections. This is the simplest mode and serves
// as the ultimate fallback.
class ModeManual : public FlightModeBase {
public:
    ModeManual() = default;

    FlightMode id() const override { return FlightMode::Manual; }
    const char* name() const override { return "Manual"; }

    bool can_enter(const AircraftState& /*state*/) const override {
        // Manual mode can always be entered
        return true;
    }

    void enter() override {
        // No state to initialize
    }

    void exit() override {
        // No state to clean up
    }

    Result<ControlSurfaces> update(const AircraftState& /*state*/,
                                   const RCInput& rc,
                                   float /*dt*/) override {
        if (!rc.is_valid()) {
            return ErrorCode::RCLost;
        }

        ControlSurfaces surfaces;
        surfaces.aileron = rc.roll();
        surfaces.elevator = rc.pitch();
        surfaces.throttle = rc.throttle();
        surfaces.rudder = rc.yaw();
        surfaces.flaps = 0.0f;
        surfaces.clamp();

        return surfaces;
    }
};

} // namespace fw_uav
