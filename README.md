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
<video src="videos/stacked_output.mp4" controls width="320" height="240">
</video>

## How to run:
1) Clone the repository \
`git clone https://github.com/IndraNeelMulakaloori/ENPM661_FINAL_PROJECT.git`
2) For every algorithm, such as RRT, RRT*, RRT_star_N(fixed), RRT_star_N_dynamic, run the scripts 
```
python3 RRT.py
python3 RRT_star.py
python3 RRT_star_N.py
python3 RRT_star_N_dynamic.py
```
3) During the execution, you might see the terminal showing the logs of the output i.e
```
New Node: 177, 137
Nearest Node: 163, 90
New Node: 211, 102
Nearest Node: 53, 245
New Node: 98, 264
Nearest Node: 163, 90
New Node: 184, 135
Nearest Node: 163, 90
New Node: 211, 102
Nearest Node: 163, 90
New Node: 212, 91
Nearest Node: 163, 90
New Node: 209, 107
Nearest Node: 163, 90
New Node: 211, 102
Nearest Node: 56, 77
New Node: 57, 77
Nearest Node: 163, 56
New Node: 212, 56
Nearest Node: 163, 90
New Node: 209, 108
Nearest Node: 163, 90
New Node: 212, 95
Nearest Node: 163, 90
New Node: 212, 97
Nearest Node: 163, 90
New Node: 212, 93
Nearest Node: 52, 128
New Node: 83, 127
Nearest Node: 53, 218
New Node: 61, 215
Nearest Node: 163, 90
New Node: 211, 102
Nearest Node: 163, 90
New Node: 211, 102
Nearest Node: 163, 56
New Node: 211, 43
Nearest Node: 163, 90
New Node: 211, 102
Nearest Node: 163, 90
New Node: 212, 96
Nearest Node: 53, 148
New Node: 101, 137
Nearest Node: 163, 90
New Node: 211, 103
Failed to find a path
Finished 'plan' in 2.3171 secs
No path found.
```
with run time and result whether it could find a path

4) After execution, teh script generates video of the respective path planner.

5) Instead of user input, in this project, we created a `parameters.json` file to provide inputs such as 

**Input Parameters** :

```
"start": {
        "x": 34,
        "y": 70
    },
    "goal": {
        "x": 510,
        "y": 180
    },
"clearance": 25
``` 
**HyperParameters**
```
    "clearance": 25,
    "max_iterations": 10000,
    "step_size": 50,
    "goal_sample_rate" : 0.15,
    "goal_threshold" : 3
```
to perform several tests.\
6) `stacked_output.py` generates the stacked output of the videos for comaprison between performance of the alogrithms.

## Gazebo Simulation
1) Change Directory to `project3_ws`
`cd project3_ws`
2) Build the workspace, export the **Turtlebot3**  model, source and launch the world
```
colcon build --symlink install
export TURTLEBOT3_MODEL=waffle
source install/setup.bash
ros2 launch turtlebot3_project3 competition_world.launch.py
```
,then Colcon build and source the workspace. launch the world in Gazebo with "export TURTLEBOT3_MODEL=waffle" and "ros2 launch turtlebot3_project3 competition_world.launch.py". change the destination for path_coordinates for it to read the waypoints for gazebo simulation in plan_and_follow.py then launch our ROS node in a seperate terminal with "ros2 run path_follow_executor plan_and_follow".use waitkey- 0 to reach the end goal node.
## Result

Compared to vanilla RRT\*, this approach:
- Reduces the number of required iterations to reach the goal (in open spaces).
- Finds smoother and more optimal paths.
- Performs better in narrow corridors and structured maps.
