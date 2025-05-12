# RRT*N 

This algorithm is an improved version of RRT* by manipulating the generation nodes using probabilistic Normal(Gaussian) distribution.

## Key Mathematical Intuition

### Traditional RRT\*
In RRT*, the random sampling is **uniform** across the entire configuration space:
$$
\vec{z}_{sample} = 
\begin{bmatrix}
\text{Uniform}(0, W) \\
\text{Uniform}(0, H)
\end{bmatrix}
$$

While exploratory, this often leads to **slow convergence** toward the goal in cluttered or large spaces.

---

### RRT*N: Gaussian Biased Sampling

RRT*N improves convergence by sampling points **closer to the optimal trajectory** using a **normal distribution** around the straight line from start to goal.

Let:
- $(\vec{z}_{start}, \vec{z}_{goal})$ be the start and goal points.
- $(\vec{L} = \vec{z}_{goal} - \vec{z}_{start})$
- $(\vec{u}_L = \frac{\vec{L}}{\|\vec{L}\|})$ be the unit vector along $( \vec{L})$
- $(\vec{n}_L = [-u_{Ly}, u_{Lx}])$ be the **normal** (perpendicular) direction to the line.

Then:
$$
[
\vec{z}_{sample} = \vec{z}_{start} + t \cdot \vec{L} + \delta \cdot \vec{n}_L
]
$$

Where:
- $( t \sim \text{Uniform}(0, 1) )$ — samples **along** the centerline.
- $( \delta \sim \mathcal{N}(0, \sigma^2) )$ — applies **Gaussian offset** **perpendicular** to the line.
- $( \sigma )$ dynamically adjusts to control exploration-vs-exploitation.

### Probability Density Function (PDF)

Sampling is guided by a Gaussian distribution centered around the path line:

$$
P(c) = \exp\left(-\frac{(c - \mu)^2}{2\sigma^2}\right)
$$

Where:
- $( c )$ is the **perpendicular distance** from a candidate point to the line $(L)$\,
- $( \mu = 0 )$, ensuring the peak of the distribution lies directly on the line,
- $( \sigma )$ is dynamically tuned based on planning progress.



## Sigma Adaptation Strategy

To balance **goal convergence** and **global exploration**, sigma $( \sigma )$ is updated dynamically:

- **Increase $( \sigma )$** when the algorithm is stuck (many failed samples).
- **Decrease $( \sigma )$** when making progress (nodes are moving toward the goal).

This makes the sampling **adaptive**, allowing RRT\*N to escape local minima while prioritizing optimal paths.

## Comparison of algorithms:
<video controls src="videos/stacked_output.mp4" title="Title"></video>
## Gazebo Simulation
clone project3_ws,then Colcon build and source the workspace. launch the world in Gazebo with "export TURTLEBOT3_MODEL=waffle" and "ros2 launch turtlebot3_project3 competition_world.launch.py". change the destination for path_coordinates for it to read the waypoints for gazebo simulation in plan_and_follow.py then launch our ROS node in a seperate terminal with "ros2 run path_follow_executor plan_and_follow".use waitkey- 0 to reach the end goal node.
## Result

Compared to vanilla RRT\*, this approach:
- Reduces the number of required iterations to reach the goal (in open spaces).
- Finds smoother and more optimal paths.
- Performs better in narrow corridors and structured maps.
