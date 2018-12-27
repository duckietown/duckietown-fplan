std_msgs/String request_id  # The request ID
std_msgs/UInt32 start_time  # The timestep of the request creation (calculate with dt to get time in seconds)
std_msgs/UInt32 pickup_time  # The timestep of pickup (0 if not picked up)
std_msgs/UInt32 end_time  # The timestep of drop-off (0 if not dropped off)
std_msgs/String start_node  # The node for pick-up (see _Map_ later)
std_msgs/String end_node  # The node for drop-off
std_msgs/String duckie_id  # The duckie that picked up the request (empty if request is waiting)
