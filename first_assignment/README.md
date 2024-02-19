GROUP 23

Andrea Ghiotto, andrea.ghiotto.3@studenti.unipd.it

Nicola Lorenzon, nicola.lorenzon.3@studenti.unipd.it

Daniele Moschetta, daniele.moschetta@studenti.unipd.it

# Run instructions
Run the following commands in different terminals:

Without Motion Control Law:

- `roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library`

- `roslaunch tiago_iaslab_simulation navigation.launch`

- `rosrun first_assignment DetectObstaclesServer`

- `rosrun first_assignment DetectObstaclesClient 11 0 270`

With Motion Control Law:

- `roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library`

- `roslaunch tiago_iaslab_simulation navigation.launch`

- `rosrun first_assignment MotionControlLawServer`

- `rosrun first_assignment DetectObstaclesMCLServer`

- `rosrun first_assignment DetectObstaclesClient 11 0 270`