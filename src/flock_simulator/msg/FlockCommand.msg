std_msgs/Header header  # Generic header
std_msgs/Float64 dt  # The timestep duration of the simulation in seconds, should not be too big (ideally ~0.05 or less)
flock_simulator/DuckieCommand[] duckie_commands  # An array for individual commands for duckies
