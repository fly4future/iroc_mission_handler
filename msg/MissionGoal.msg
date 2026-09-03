string name
uint8 frame_id
uint8 height_id
iroc_mission_handler/Waypoint[] points
uint8 terminal_action

# Optional per-robot override of the MRS trajectory generation constraints
# (https://ctu-mrs.github.io/docs/features/trajectory_generation/). Left at
# its default (override_constraints=false) the trajectory is generated using
# the UAV's currently active constraint profile, unchanged. Planners that
# don't need per-mission speed/acceleration overrides can simply leave these
# fields unset.
bool override_constraints
float64 override_max_velocity_horizontal
float64 override_max_acceleration_horizontal
float64 override_max_jerk_horizontal
float64 override_max_velocity_vertical
float64 override_max_acceleration_vertical
float64 override_max_jerk_vertical

uint8 TERMINAL_ACTION_NONE=0
uint8 TERMINAL_ACTION_LAND=1
uint8 TERMINAL_ACTION_RTH=2

uint8 FRAME_ID_LOCAL=0
uint8 FRAME_ID_LATLON=1
uint8 FRAME_ID_FCU=2

# Flag to specify the height for the points: Above ground level (AGL) or Above Mean Sea Level (AMSL)
uint8 HEIGHT_ID_AGL=0
uint8 HEIGHT_ID_AMSL=1
uint8 HEIGHT_ID_FCU=2
