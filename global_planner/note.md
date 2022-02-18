# useful notes about the functioning of the planner

- In order to change the default parameters and their variability width in rviz there is a file called `GlobalPlannerNode.cfg` that can be modified. It is also required to recompile the code afterwards for the changes to take effect.

- If the parameter `clicked_goal_alt_` is lower than the initial vertical position, specified in `global_planner_depth_camera.launch`, then the planner complains a lot.

- Tweaking too much with the resolution parameter in `global_planner_octomap.launch` and setting it too low might create problems as the planner computes a cell z pose that is a saturated variable basically. The companion parameter to this is `CELL_SCALE` in the aforementioned `.cfg` file. The value of the latter should a multiple of the resulution of octomap.

- The failure of the methiod `octomap::search` is caused by  the combianation of the resolution and the `robot_radius`, referred to as `radius` in `global_planner.cpp`, since there is a division between these two parameters in `double GlobalPlanner::getRisk(const Cell& cell)`. This operation is probably the cause of the saturation to the max of an 32-bit integer of `neighbor.zPos()`. The cause is in the division or the start of the cycle for.

# indoor navigation with global_planner questio

Theoretically this planner cannot be used inside, but with the aforementioned tweak in radius and resolution, and their ratio, it can easily detect small obstacles, such those present in the indoors. 
So, the global planner is suitable by itself for indoor navigation with its characteristics. 
The remaioning problem is that using `fake_gps` in vision mode does not seem to allow locking a home position, not allowing the wanted use.
This has to be solved to use the planner indoor, but outdoor is already fine.
