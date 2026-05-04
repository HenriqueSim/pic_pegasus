# Statistics for each individual PID controller at the acceleration tracking level
PidStatistic[3] pid_statistics
# Statistics for the overall acceleration tracking
float64[3] rotation_error
# Desired angular rate
float64[3] desired_angular_rate
# Total Thrust asked to the system (in N)
float64 thrust_reference
# Attitude rate asked to the system (in deg/s)
float64[3] attitude_rate_reference
# Current attitude of the drone (in deg)
float64 state_roll
float64 state_pitch
float64 state_yaw
# Desired attitude of the drone (in deg)
float64 desired_roll
float64 desired_pitch
float64 desired_yaw