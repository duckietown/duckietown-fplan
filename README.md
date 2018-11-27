# fleetplanning18
Fleet planning simulator developed as a AMOD course project 2018

## Flock simulator
```
rosrun flock_simulator flock_simulator_node.py
```
Receives commands on the topic `/flock_simulator/state` and updates the state accordingly which is then published on `/flock_simulator/commands`.

The duckies execute the commands (linear and angular velocity) that are tagged with their ID. If a duckie does not receive any commands, it drives around randomly (on rails). Once a duckie has received commands, it stops driving around and stops when there are no more commands. To force the "on rails" behavior, set the `on_rails` flag in the commands to `True` (unexpected behavior when steering "out of rails", to be fixed).

The state is published as an array of `DuckieState`. The following information is contained:
```
duckie_id  # The ID of the duckie
on_service  # Boolean if a duckie has been picked up and is being dropped off, relevant for fleet planning
in_fov  # Array of duckie ids that are in its field of view
pose  # Duckie's pose
velocity  # Duckie's velocity (currently set as last command)
```

The uncontrolled duckies will slow down if there is another duckie in front of them, unless they are on an intersection. Collisions and traffic rules are not yet implemented.

## Flock simulator GUI
WIP

## Flock planner
```
rosrun flock_planner flock_planner_node.py
```
It is currently publishing empty messages to `/flock_simulator/commands`. To try out the simulator, simply run both nodes, the duckies will be driving around randomly.
