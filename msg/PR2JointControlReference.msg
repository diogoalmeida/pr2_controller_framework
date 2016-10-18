sensor_msgs/JointState commanded_joint_state  # Taken as input. Uses the position and velocity fields as references, referenced by name
bool use_current_position # Ignores the position field of the reference topic and uses the current values instead
