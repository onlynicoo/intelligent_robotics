GROUP 23

Nicola Lorenzon, nicola.lorenzon.3@studenti.unipd.it

Daniele Moschetta, daniele.moschetta@studenti.unipd.it

# Run instructions

Run the following commands in different terminals:

Without bonus:

- `roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables`

- `roslaunch tiago_iaslab_simulation apriltag.launch`

- `roslaunch tiago_iaslab_simulation navigation.launch`

- `roslaunch second_assignment pick_and_place.launch`

- `rosrun second_assignment nodeA`

With bonus:

- `roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables`

- `roslaunch tiago_iaslab_simulation apriltag.launch` 

- `roslaunch tiago_iaslab_simulation navigation.launch`

- `roslaunch second_assignment pick_and_place.launch`

- `rosrun second_assignment nodeA_bonus`

The launch file `pick_and_place.launch` launches the ROS nodes `DetectObstaclesServer`, `nodeC`, `nodeB` and `human_node`. We chose not to include the other launch files inside this one, because otherwise it would be difficult to properly read our logs.

To run with a specific ID order change the last command with `rosrun second_assignment nodeA 1 2 3`.