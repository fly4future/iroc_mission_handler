# Task identification
string type                     # Type of task
string parameters               # Parameters for the specific task

# Subtask execution details
bool continue_without_waiting   # Whether to continue mission without waiting for this subtask to complete
bool stop_on_failure            # Whether to stop the mission if this task fails
uint8 max_retries               # Maximum number of retries for this task if it fails
float64 retry_delay             # Delay before retrying the task, in seconds