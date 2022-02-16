# useful notes about the functioning of the planner

- In order to change the default parameters and their variability width in rviz there is a file called `GlobalPlannerNode.cfg` that can be modified. It is also required to recompile the code afterwards for the changes to take effect.

- If the parameter `clicked_goal_alt_` is lower than the initial vertical position, specified in `global_planner_depth_camera.launch`, then the planner complains a lot.

- Tweaking too much with the resolution parameter in `global_planner_octomap.launch` and setting it too low might create problems as the planner computes a cell z pose that is a saturated variable basically. The companion parameter to this is `CELL_SCALE` in the aforementioned `.cfg` file. The value of the latter should a multiple of the resulution of octomap.
