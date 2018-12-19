# fleetplanning18
Fleet planning simulator developed as a AMOD course project 2018

## Flock simulator
Edit the parameters in `flock_simulator/flock_simulator.launch` and launch it with:
```
roslaunch flock_simulator flock_simulator.launch
```

### State
The state of the simulator is published on `/flock_simulator/state` with every update (triggered by messages on `/flock_simulator/commands`). The message contains following information:
- `std_msgs/Header header` Generic header
- `flock_simulator/DuckieState[] duckie_states` An array of duckie states
- `flock_simulator/Request[] requests` An array of requests that have not been filled (either waiting or being driven around)
- `flock_simulator/Request[] filled_requests` An array of filled requests (picked up and dropped off again)

#### Duckie state
The duckie state message contains following information:
- `std_msgs/String duckie_id` The duckie's ID
- `std_msgs/String[] in_fov` An array of all the duckies that it can see in its field of view
- `std_msgs/String status` For fleetplanning ('IDLE', 'REBALANCING', 'DRIVINGTOCUSTOMER', 'DRIVINGWITHCUSTOMER')
- `std_msgs/String lane` Current lane the duckie is on
- `std_msgs/UInt8 collision_level` 0: No collision, 1: collision
- `geometry_msgs/Pose2D pose` Current pose
- `geometry_msgs/Twist velocity` Current velocity

#### Requests
A request message is defined by:
- `std_msgs/String request_id` The request ID
- `std_msgs/UInt32 start_time` The *timestep* of the request creation (calculate with `dt` to get time in seconds)
- `std_msgs/UInt32 pickup_time` The timestep of pickup (0 if not picked up)
- `std_msgs/UInt32 end_time` The timestep of drop-off (0 if not dropped off)
- `std_msgs/String start_node` The node for pick-up (see _Map_ later)
- `std_msgs/String end_node` The node for drop-off
- `std_msgs/String duckie_id` The duckie that picked up the request (empty if request is waiting)

### Commands
A command can be issued on the topic `/flock_simulator/commands`, which also triggers a timestep for the simulation (the simulator does not do anything if there are no commands). A command for the flock has the following structure:
- `std_msgs/Header header` Generic header
- `std_msgs/Float64 dt` The timestep duration of the simulation in seconds, should not be too big (ideally ~0.05 or less)
- `flock_simulator/DuckieCommand[] duckie_commands` An array for individual commands for duckies

#### Duckie command
The command message for a single duckie is defined as follows:
- `std_msgs/String duckie_id` The duckie this command is meant for
- `std_msgs/Bool on_rails` Set to `True` if the command is defined by the `path` below, set to `False` if the command is given as velocities in `velocity_command` below
- `std_msgs/String[] path` An array of nodes that define the path
- `std_msgs/String request_id` The request ID the duckie should serve (picks up or drops off if driving by)
- `geometry_msgs/Twist velocity_command` Velocity commands (linear velocity in `linear.x`, angular velocity in `angular.z`)
Commands can be given as a path or as velocites. For the former, `on_rails` should be set to `True` in which case the duckie will ignore `velocity_command`, and vice versa.

The "on-rails" duckies follow very simple traffic rules, such as keeping a certain distance from duckies in front and giving right of way to duckies approaching intersections from the right. Also, they stop in front of intersections if there is already a duckie on it. *Note:* The current rules occasionally lead to collisions.

### Visualization
The simulation is visualized using rviz. The duckies are shown as meshes of their real-life counterparts, requests shown as spheres (yellow for pick-up, green for drop-off). *Note:* Due to a bug correctly dropped-off requests (green spheres) persist and only disappear after a new request appears).

### Internal structure
The simulator represents duckies as instances of the class `Duckiebot` and requests as instances of the class `Request`. A single instance of class `StateManager` is used to represent the entire state of the simulation and coordinate the interaction between duckies, requests and the map. The class `DuckietownMap` contains relevant information of the map and methods for extracting information.

## Flock planner
The flock planner interacts with the simulator and publishes commands on it.

### Dispatcher
The dispatcher receives the state of the simulation and returns paths for each duckie. The currenlty implemented dispatcher is deliberately kept very rudimentary, so it can be improved by users (and in the future hopefully AIDO participants) and tested.


