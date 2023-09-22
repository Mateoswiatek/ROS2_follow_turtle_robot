# Follow Turtle Robot
## Introduction and brief description
The project involves spawning new turtles at random locations on a prepared map 2D, with one designated robot 'collecting' them either from the nearest or the oldest one added. Topics, Services, and Parameters were utilized. Two Launch Files have been prepared. The project is fully autonomous, with the robot dynamically calculating the route and destination. It represents the culmination and utilization of all the fundamental concepts related to ROS2, opening the door to more complex projects. Currently, as I delve into the realm of ROS2, I am learning Gazebo and related topics.

My previous smaller projects in ROS2:
- [collection of projects](https://github.com/Mateoswiatek/ROS2_other_small_projects)
- [roulette_with_parameters](https://github.com/Mateoswiatek/ROS2_roulette_with_parameters_and_server)

## Technologies Used

Utilized Features:
- Packages,
- Custon Topics,
- Custom Services,
- Custom Interfaces (Msg and Srv)
- Threads,
- Parameters,
- Launch Files,
- Bags

## Full Description
The first Node is **"turtle_spawner"** responsible for managing the spawned turtles. It generates a new turtle in a random location, and the frequency is set using a parameter. Additionally, the maximum number of turtles on the board is set via a corresponding parameter. Since the turtles do not move (this is one of the future functionalities), the Spawner publishes the current turtle's state through a Topic only when a new one appears (it could just as well publish the position of the newest one only). It also provides a Service through which the robot can capture and eliminate a turtle. This Node essentially "reads" from the environment (turtlesim) as interpreted by the Robot. The Node simulates the robot's sensors.

The second Node is **"Robot"** which receives the array of living turtles from **"turtle_spawner"** via a Topic and also gets its current position from turtlesim_node (the environment) via a Topic. It calculates the route to the target in real-time, with the target type (oldest or nearest) being set using a parameter. The aggressiveness of movements can also be adjusted via parameters. It publishes the current linear and angular velocity values to the environment where the entire simulation takes place. The Robot dynamically determines its goal and computes linear and angular velocity values. The closer it is to the target, the more it slows down. When it reaches a specified distance from the target (which can also be set using a parameter), it sends a request to **"turtle_spawner"** to indicate that it has captured the target.

The entire project showcases the beauty of ROS2 and the potential for personalizing the entire program. It has sparked numerous new ideas for ROS2 applications. Understanding these ROS2 "fundamentals" and applying them to create this project motivates me to delve deeper into knowledge and explore more advanced topics. Proficiency in ROS2 is an exciting path, especially considering the possibility of connecting robots with AI, an area I will soon be delving into as well. ROS2 serves as a perfect bridge, combining my existing knowledge in mechatronics and electronics with future AI expertise. Such a combination will undoubtedly lead to innovative projects.


## Potential for Further Development / Ideas 
- Adding a "random starting coordinates" parameter for the main robot.
- Introducing multiple turtles with various speed and angle parameters for catching them, creating competition, and providing feedback for adjusting velocity parameters.
- Adding a parameter for "immediate orientation towards motion," resulting in straight-line paths.
- Implementing motion for each turtle, which would require an additional node responsible for controlling the living turtles and sending movement commands to each of them. We would receive a list of living turtles from **"turtle_spawner"** and would need to create publishers on separate threads for the relevant topics. In case of capture, **"turtle_spawner"** would have to send information to terminate this thread.
- Transitioning to a 3D environment, along with challenges for the robot.
- Incorporating AI algorithms for various tasks, solutions, and problems, making it an ideal environment for smaller AI projects
