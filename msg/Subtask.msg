# Task identification
string type                     # Type of task
string parameters               # Parameters for the specific task

# Subtask execution details
bool wait_for_completion        # Whether the mission should wait for this task to complete
bool stop_on_failure            # Whether to stop the mission if this task fails
uint8 max_retries               # Maximum number of retries for this task if it fails
float64 retry_delay             # Delay before retrying the task, in seconds