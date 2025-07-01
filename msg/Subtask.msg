# Task type constants
uint8 TASK_WAIT = 0
uint8 TASK_GIMBAL = 1

# Task identification
uint8 type                      # Type of task (from constants above)
string parameters               # Parameters for the specific task

# Task timing
float64 max_duration            # Maximum duration in seconds, 0 for instantaneous tasks
bool wait_for_completion        # Whether the mission should wait for this task to complete