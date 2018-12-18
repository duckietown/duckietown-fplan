# fleetplanning18
Fleet planning simulator developed as a AMOD course project 2018

## Flock simulator
To launch the simulator, visualization and dummy planner (to trigger timesteps):
```
roslaunch flock_simulator flock_simulator.launch
```
Receives commands on the topic `/flock_simulator/commands` and updates the state accordingly which is then published on `/flock_simulator/state`.

The duckies execute the commands (linear and angular velocity) that are tagged with their ID. If a duckie does not receive any commands, it drives around randomly (on rails). Once a duckie has received commands, it stops driving around and stops when there are no more commands. To fix slight deviations from "on rails" behavior, set the `on_rails` flag in the commands to `True` (will ignore if clearly not on rails).

The state is published as an array of `DuckieState`. The following information is contained:
```
duckie_id  # The ID of the duckie
status  # Taxi status of duckie, relevant for fleet planning
in_fov  # Array of duckie ids that are in its field of view
collision_level  # 0: no collision, 1: collision with duckiebot, 2: out of lane
pose  # Duckie's pose
velocity  # Duckie's velocity (currently set as last command)
```

The uncontrolled duckies will slow down if there is another duckie in front of them, traffic rules are yet to be implemented.

## Flock planner
```
rosrun flock_planner flock_planner_node.py
```
It is currently publishing empty messages to `/flock_simulator/commands`. To try out the simulator, simply run both nodes, the duckies will be driving around randomly.
