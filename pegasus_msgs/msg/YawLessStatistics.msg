# PID statistics for x, y, z axes
pegasus_msgs/PidStatistic[3] pid_statistics

# Attitude errors and references
float64[3] rotation_error
float64[3] desired_angular_rate
float64[3] attitude_rate_reference

# Thrust
float64 thrust_reference

# Current attitude [deg]
float64 state_roll
float64 state_pitch
float64 state_yaw

# Desired attitude [deg]
float64 desired_roll
float64 desired_pitch
float64 desired_yaw