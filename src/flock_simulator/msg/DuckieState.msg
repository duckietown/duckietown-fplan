std_msgs/String duckie_id  # The duckie's ID
std_msgs/String[] in_fov  # An array of all the duckies that it can see in its field of view
std_msgs/String status  # For fleetplanning ('IDLE', 'REBALANCING', 'DRIVINGTOCUSTOMER', 'DRIVINGWITHCUSTOMER')
std_msgs/String lane  # Current lane the duckie is on
std_msgs/UInt8 collision_level  # 0: No collision, 1: collision
geometry_msgs/Pose2D pose  # Current pose
geometry_msgs/Twist velocity  # Current velocity
