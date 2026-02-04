#include "fw_uav/control/pid.h"

// PID implementation is header-only for now since it's templated and inline-friendly.
// This file exists for potential future non-inline implementations and to ensure
// the translation unit is compiled during build validation.

namespace fw_uav {

// Explicit instantiation could go here if needed for code size optimization

} // namespace fw_uav
