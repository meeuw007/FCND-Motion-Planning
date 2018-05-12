Explanation of provided code


The last line of code starts the drone (in the “_main_” part of the code).

The state_callback function elevates the drone into the different states: At first the drone goes to the arming_transition (because the ‘self.in_mission’ attribute is initialized to true in line 32, and the first state of the drone is initialized to Manual).

Then (in the Arming state) the planning fuction is called, and the drone goes to the Planning state. Here the target altitude and safe distance for the drone to keep is set, and data from the colliders csv is read. From this data a grid is made calling the function create_grid from planning_utils.

create_grid makes a matrix representing a 2D configuration space with 0’s representing free space (including space a safe distance above obstacles), and 1’s representing obstacles (or space too close to obstacles).

The start position of the plan is set to the center of the map and the goal position is set to the start position plus 10 grid cells to the right, and 10 grid cells upwards.

The function a_star from planning_utils is called to plan a path from goal to start, which is then converted to waypoints, which are sent to the simulator.

a_star finds a path by expanding the gridcells one by one, checking for free space, keeping track of all possible routes and respective costs, and returning the lowest cost path.

The state_callback function elevates the drone from the Planning state to the Take off state.

When the drone is within 95% of the target altitude the local_position_callback function elevates the drone into the Waypoint state.

The waypoint_transition functions ‘pops’ the first waypoint of the list and sets it as target position.

The local_position_callback function calls the waypoint_transition function again when the norm of the local_position vector is within 1.0 of the norm of the target_position vector, and the waypoint_transition function pops the next waypoint.

In the starter code the goal position is ten cells to the right, and ten cells up. Sinds the a_star function in planning_utils can only go up and right (not diagonal) the drone will follow a zig zag path.

When all waypoints are used, the local_position_callback function calls the landing_transition function and the drone goes into the Landing state.

Finally the velocity_callback function transitions the drone into the disarming state.


Implementation of Planning Algorithm


1. The python csv module (line 3) is imported and used to put the first row of colliders.csv in an array. I then simply assign lat0 to the first element of this array, and lon0 to the second.

With these values self.set_home_position() sets the home position (line 151).

2. Using the geodetic home coordinates found, and using self.latitude/self.longitude for the drone coordinates, in combination with the global_to_local() function I find the local position relative to global home.

3. In line 184 I set the grid_start to the local position of the Drone.

4. In line 193 and 194 the grid_goal is set to the corner of Washington Str and Battery Str (arbitrary choice). In the code is two lines provide to set the grid_goal back to the home position (uncomment line 197 and 198 (and comment line 193 and 194 of course)). grid_goal can be set to any latitude and longitude within the map.

5. To include diagonal motion I first define 4 more actions in the class Action (in planning_utils.py). I import the python module math, and use it to calculate the square root of 2 which I assign as cost to the four new actions (NorthWest, SouthWest, SouthEast, and NorthEast). I also expand the if statements in valid_actions, to check if the four new actions, when implemented, run into an obstacle.

6. To prune the path I added the function prune_path(path) to planning_utils.py. Also added are point(p) and collinearity_check() which are used by prune_path. In prune_path three waypoints are checked if they are in line by checking if the determinant of the matrix that includes the coordinates of these three points is 0 (or nearly 0). Then one of those points is removed and the remaining two checked against another third one, and so on. prune_path is called in motion_planning.py in line 227





