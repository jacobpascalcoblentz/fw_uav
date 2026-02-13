#include "fw_uav/modes/flight_mode_manager.h"

namespace fw_uav {

bool FlightModeManager::register_mode(FlightModeBase* mode) {
    if (mode == nullptr) {
        return false;
    }

    int index = static_cast<int>(mode->id());
    if (index < 0 || index >= MAX_MODES) {
        return false;
    }

    if (modes_[index] != nullptr) {
        return false;  // Slot already occupied
    }

    modes_[index] = mode;
    return true;
}

Result<void> FlightModeManager::request_mode(FlightMode mode, const AircraftState& state) {
    int index = static_cast<int>(mode);
    if (index < 0 || index >= MAX_MODES) {
        return ErrorCode::ModeChangeRejected;
    }

    FlightModeBase* target = modes_[index];
    if (target == nullptr) {
        return ErrorCode::ModeChangeRejected;
    }

    if (!target->can_enter(state)) {
        return ErrorCode::ModeChangeRejected;
    }

    // Exit current mode
    if (active_mode_ != nullptr) {
        active_mode_->exit();
    }

    // Enter new mode
    active_mode_ = target;
    current_mode_id_ = mode;
    active_mode_->enter();

    return Result<void>();
}

Result<ControlSurfaces> FlightModeManager::update(const AircraftState& state,
                                                   const RCInput& rc,
                                                   float dt) {
    if (active_mode_ == nullptr) {
        return ErrorCode::ControlNotInitialized;
    }

    return active_mode_->update(state, rc, dt);
}

FlightModeBase* FlightModeManager::mode_ptr(FlightMode mode) const {
    int index = static_cast<int>(mode);
    if (index < 0 || index >= MAX_MODES) {
        return nullptr;
    }
    return modes_[index];
}

bool FlightModeManager::has_mode(FlightMode mode) const {
    return mode_ptr(mode) != nullptr;
}

} // namespace fw_uav
