string name
string message
# Next waypoint information
uint16 goal_idx                     # Index of the next waypoint the drone is flying to
float64 distance_to_closest_goal    # Distance (in meters) to the next waypoint
float64 goal_estimated_arrival_time # Estimated time of arrival (in seconds) to the next waypoint
float64 goal_progress               # Progress towards the next waypoint as a percentage (0.0 - 100.0)

# Mission completion information
float64 distance_to_finish            # Remaining distance (in meters) to the mission finish point
float64 finish_estimated_arrival_time # Estimated time of arrival (in seconds) to the mission finish point
float64 mission_progress              # Overall mission progress as a percentage (0.0 - 100.0)
