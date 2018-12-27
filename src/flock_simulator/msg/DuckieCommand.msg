std_msgs/String duckie_id  # The duckie this command is meant for
std_msgs/Bool on_rails  # Set to True if the command is defined by the path below, set to False if the command is given as velocities in velocity_command below
std_msgs/String[] path  # An array of nodes that define the path
std_msgs/String request_id  # The request ID the duckie should serve (picks up or drops off if driving by)
geometry_msgs/Twist velocity_command  # Velocity commands (linear velocity in linear.x, angular velocity in angular.z)
