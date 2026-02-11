string name 
uint8 frame_id
uint8 height_id
iroc_mission_handler/Waypoint[] points
uint8 terminal_action

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
