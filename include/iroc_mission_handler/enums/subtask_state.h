#pragma once
#include <cstdint>

#undef X_ENUM_NAME
#undef X_ENUM_BASE_TYPE
#undef X_ENUM_SEQ

#define X_ENUM_NAME subtask_state_t
#define X_ENUM_BASE_TYPE uint8_t
#define X_ENUM_SEQ (IDLE)(RUNNING)(COMPLETED)(FAILED)

namespace iroc_mission_handler
{

#include <iroc_common/helpers/enum_macros.hpp>

DEFINE_ENUM_WITH_CONVERSIONS(X_ENUM_NAME, X_ENUM_BASE_TYPE, X_ENUM_SEQ)

} // namespace iroc_mission_handler
