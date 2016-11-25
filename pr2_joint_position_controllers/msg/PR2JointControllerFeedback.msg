# Feedback: information that may be used to debug the controller
string[] joint_name
float64[] commanded_effort # Values sent to the robot
float64[] position
float64[] position_error
float64[] velocity
float64[] velocity_error
float64 position_error_norm
float64 velocity_error_norm
float64 effort_single
float64 position_feedback_norm  # "position effort"
