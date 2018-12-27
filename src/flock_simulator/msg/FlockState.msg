std_msgs/Header header  # Generic header
flock_simulator/DuckieState[] duckie_states  # An array of duckie states
flock_simulator/Request[] requests  # An array of requests that have not been filled (either waiting or being driven around)
flock_simulator/Request[] filled_requests  # An array of filled requests (picked up and dropped off again)
