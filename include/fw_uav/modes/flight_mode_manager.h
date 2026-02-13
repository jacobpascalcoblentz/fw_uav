#pragma once

#include <cstdint>
#include "fw_uav/core/types.h"
#include "fw_uav/core/result.h"

namespace fw_uav {

// Abstract base class for all flight modes
//
// Each flight mode implements this interface. The mode manager calls
// can_enter() before transitioning, enter()/exit() on transitions,
// and update() every control loop iteration.
class FlightModeBase {
public:
    virtual ~FlightModeBase() = default;

    // Identity
    virtual FlightMode id() const = 0;
    virtual const char* name() const = 0;

    // Lifecycle
    virtual bool can_enter(const AircraftState& state) const = 0;
    virtual void enter() = 0;
    virtual void exit() = 0;

    // Main control update
    // @param state   Current aircraft state
    // @param rc      Current RC input
    // @param dt      Time step in seconds
    // @return Control surface commands on success, error code on failure
    virtual Result<ControlSurfaces> update(const AircraftState& state,
                                           const RCInput& rc,
                                           float dt) = 0;
};

// Flight mode manager: state machine managing transitions between modes
//
// Holds an array of registered mode pointers and manages the active mode.
// On each control loop tick, call update() which delegates to the active
// mode. Mode transitions are requested via request_mode() and only
// succeed if the target mode's can_enter() returns true.
class FlightModeManager {
public:
    static constexpr int MAX_MODES = static_cast<int>(FlightMode::NumModes);

    FlightModeManager() = default;

    // Register a mode. Returns false if the slot is already occupied.
    bool register_mode(FlightModeBase* mode);

    // Request a mode transition. Returns an error if the mode is not
    // registered or if can_enter() returns false.
    Result<void> request_mode(FlightMode mode, const AircraftState& state);

    // Run the active mode's update
    Result<ControlSurfaces> update(const AircraftState& state,
                                   const RCInput& rc,
                                   float dt);

    // Query
    FlightMode current_mode() const { return current_mode_id_; }
    FlightModeBase* current_mode_ptr() const { return active_mode_; }
    FlightModeBase* mode_ptr(FlightMode mode) const;
    bool has_mode(FlightMode mode) const;

private:
    FlightModeBase* modes_[MAX_MODES]{};
    FlightModeBase* active_mode_{nullptr};
    FlightMode current_mode_id_{FlightMode::Manual};
};

} // namespace fw_uav
