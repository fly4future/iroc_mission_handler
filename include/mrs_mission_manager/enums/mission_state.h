#pragma once
#include <cstdint>
#include <mrs_robot_diagnostics/enums/enum_helpers.h>

#undef X_ENUM_NAME
#undef X_ENUM_BASE_TYPE
#undef X_ENUM_SEQ

#define X_ENUM_NAME       mission_state_t
#define X_ENUM_BASE_TYPE  uint8_t
#define X_ENUM_SEQ                            \
                          (IDLE)              \
                          (TAKEOFF)           \
                          (FLYING_TO_START)   \
                          (EXECUTING)         \
                          (PAUSED)            \
                          (RTL)               \
                          (LAND)           

namespace mrs_mission_manager
{

DEFINE_ENUM_WITH_CONVERSIONS(X_ENUM_NAME, X_ENUM_BASE_TYPE, X_ENUM_SEQ)

}
