# Subtask message for robot missions

# Task type constants
uint8 TASK_NONE = 0
uint8 TASK_WAIT = 1
uint8 TASK_GIMBAL = 2

# Execution point constants
uint8 EXEC_AT_WAYPOINT = 0      # Execute when reaching the waypoint
uint8 EXEC_DURING_PATH = 1      # Execute while moving between waypoints
uint8 EXEC_CONTINUOUS = 2       # Execute continuously

# Task identification
string id                       # Unique identifier for this subtask
uint8 task_type                 # Type of task (from constants above)
uint8 execution_point           # When to execute the task

# Task parameters - using flexible field to accommodate different task types
# Format is specific to each task_type and should be parsed accordingly
string parameters               # JSON-formatted parameters for the specific task

# Task timing
float64 duration                # Duration in seconds, 0 for instantaneous tasks
bool wait_for_completion        # Whether the mission should wait for this task to complete



#TODO Check Nav behavior tree