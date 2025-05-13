## DIfference 
## RRT*N is a variant of RRT* that works on genration of nodes instead of directly feeding to the algorithm
## Basically this algorithm forces to gnerate nodes in a that are The algorithm main difference is how the nodes are
# generated. However, that small change has huge effect on
# the outcome. The points that would have been directly
# fed to the algorithm in a classic RRT∗ are instead manip-
# ulated first in RRT∗ N before they are used. The points
# are limited to be created along the line L and are then
# deviated from it based on the specified Gaussian distri-
# bution. The probability law used to generate the nodes is
# the normal distribution, given by Equation (2)
# 


import numpy as np
import cv2
import json
import time
import functools
HEIGHT = 300 # cm
WIDTH = 600 # cm
WRADIUS = .033
RRADIUS = .22
WDIS = .287

# Timer decorator to measure execution time of functions
def timer(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.perf_counter()  # Start time
        result = func(*args, **kwargs)  # Execute the wrapped function
        end_time = time.perf_counter()  # End time
        run_time = end_time - start_time  # Calculate runtime
        print(f"Finished {func.__name__!r} in {run_time:.4f} secs")
        return result  # Return the result of the wrapped function
    return wrapper

        
def gen_obstacle_map():
    # Set the height and width of the image in pixels.
    height = HEIGHT
    width = WIDTH
    # Create blank canvas.
    obstacle_map = np.zeros((height,width,3), dtype=np.uint8 )
    
    # Define polygons for rectangle obstacles.
    def l_obstacle1(x,y):
        return (100 <= x <= 110) and (0 <= y <= 200)
    
    def l_obstacle2(x,y):
        return (210 <= x <= 220) and (100 <= y <= 300)
    
    def l_obstacle3(x,y):
        return (320 <= x <= 330) and (0 <= y <= 100)
    
    def l_obstacle4(x,y):
        return (320 <= x <= 330) and (200 <= y <= 300)
    
    def l_obstacle5(x,y):
        return (430 <= x <= 440) and (0 <= y <= 200)
  
    # For every pixel in the image, check if it is within the bounds of any obstacle.
    # If it is, set it's color to red.
    for y in range(height):
        for x in range(width):
            if (l_obstacle1(x, y) or l_obstacle2(x,y) or l_obstacle3(x,y) or l_obstacle4(x,y) 
                or l_obstacle5(x,y)):
                obstacle_map[y, x] = (0, 0, 255) 
            

    # The math used assumed the origin was in the bottom left.
    # The image must be vertically flipped to satisy cv2 convention. 
    return np.flipud(obstacle_map)

def expand_obstacles(image, radius):

    # Convert image to HSV.
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Define color mask for red and create grayscale image.
    lower_red = np.array([0, 200, 200])
    upper_red = np.array([25, 255, 255])
    obstacle_mask = cv2.inRange(hsv, lower_red, upper_red)
    
    # Create circular structuring element for expansion
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
    # Apply kernel to get dilation around all elements.
    expanded_mask = cv2.dilate(obstacle_mask, kernel, iterations=1)

    # Apply dilation to all of the borders.
    h, w = expanded_mask.shape
    expanded_mask[:radius+1, :] = 255  # Top border
    expanded_mask[h-radius:, :] = 255  # Bottom border
    # expanded_mask[:, :radius+1] = 255  # Left border
    # expanded_mask[:, w-radius:] = 255  # Right border
    
    # Create the output image and apply color orange to all obstacle and clearance
    # pixels.
    output_image = image.copy()
    output_image[np.where(expanded_mask == 255)] = [0, 165, 255]  # Color orange
    
    # Restore original red pixels. This creates an image with red obstacles,
    # and orange clearance zones. 
    output_image[np.where(obstacle_mask == 255)] = [0, 0, 255]  
    
    return output_image, expanded_mask
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.node_cost = 0
        self.parent = None
class RRT:
    def __init__(self, start, goal, obstacle_map, visualize_map,max_iter, step_size, goal_sample_rate, goal_threshold):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate  # probability of sampling the goal
        self.goal_threshold = goal_threshold
        self.obstacle_map = obstacle_map
        self.visualize_map = visualize_map
        self.height, self.width = obstacle_map.shape
        self.tree = [self.start]
        self.sigma = 0.1 * np.linalg.norm([self.width, self.height])
        self.sigma_min = 0.01 * np.linalg.norm([self.width, self.height])
        self.sigma_max = max(self.width, self.height)
        self.progress_buffer = []
        self.failure_buffer = []
        self.closest_node = None
        self.closest_node_distance = float('inf')


    def sample_random_point(self):
        ## Too high a goal_sample_rate (e.g. > 20%) might bias the search too much and ruin exploration.
        ## Recommended range: 5% to 15% for balance between exploration and goal convergence.
        random_sampling = np.random.rand()
        if random_sampling < self.goal_sample_rate:
            return self.goal
        else:
            return self.probability_density_function()

    def update_sigma(self):
        # Initialize these once in your class __init__
        if not hasattr(self, 'sigma'):
            self.sigma = 0.1 * np.linalg.norm([self.width, self.height])
            self.sigma_min = 0.01 * np.linalg.norm([self.width, self.height])
            self.sigma_max = max(self.width, self.height)
            self.progress_buffer = []
            self.failure_buffer = []

        alpha = 1.02  # For increasing sigma
        beta = 0.98  # For decreasing sigma
        M = 20       # Rolling window size

        recent_progress = sum(self.progress_buffer[-M:]) / max(1, len(self.progress_buffer[-M:]))
        recent_failures = sum(self.failure_buffer[-M:]) / max(1, len(self.failure_buffer[-M:]))

        if recent_failures > .8:
            self.sigma = min(self.sigma_max, self.sigma * alpha)
            print(f"Sigma updated to {self.sigma} based on recent progress: {recent_progress} and failures: {recent_failures}")
        
        elif recent_progress > 0.1:
            self.sigma = max(self.sigma_min, self.sigma * beta)
            print(f"Sigma updated to {self.sigma} based on recent progress: {recent_progress} and failures: {recent_failures}")
        elif recent_progress < 0.1:
            self.sigma = min(self.sigma_max, self.sigma * alpha)
            print(f"Sigma updated to {self.sigma} based on recent progress: {recent_progress} and failures: {recent_failures}")

    def probability_density_function(self):
        success = False

        while not success:
            ## 1) Compute the line vector from start to goal
            line_vec = np.array([self.goal.x - self.start.x, self.goal.y - self.start.y])

            ## 2) Normalize the line vector
            line_length = np.linalg.norm(line_vec)
            if line_length == 0:
                return Node(self.start.x, self.start.y)
            line_unit = line_vec / line_length

            ## 3) Perpendicular vector to the line
            line_normal = np.array([-line_unit[1], line_unit[0]])

            ## 4) Compute t_min and t_max for both directions (ray-box intersection in both directions)
            def compute_bounds(start_coord, dir_component, bound):
                if dir_component == 0:
                    return -np.inf, np.inf
                t1 = -start_coord / dir_component
                t2 = (bound - 1 - start_coord) / dir_component
                return min(t1, t2), max(t1, t2)

            t_min_x, t_max_x = compute_bounds(self.start.x, line_unit[0], self.width)
            t_min_y, t_max_y = compute_bounds(self.start.y, line_unit[1], self.height)

            t_min = max(t_min_x, t_min_y)
            t_max = min(t_max_x, t_max_y)

            if t_max < t_min:
                return Node(self.start.x, self.start.y)  # No valid extension

            ## 5) Sample a point along the line in both directions
            t = np.random.uniform(t_min, t_max)
            point_on_line = np.array([self.start.x, self.start.y]) + t * line_unit

            ## 6) Apply Gaussian offset perpendicular to the line
            #sigma = line_length // 2  # Or dynamically adjust if needed
            offset_distance = np.random.normal(0, self.sigma)

            ## 7) Bias the point along the normal vector
            biased_point = point_on_line + offset_distance * line_normal
            x, y = int(biased_point[0]), int(biased_point[1])

            ## 8) Check if the point is within the bounds of the map
            if x < 0 or x >= self.width or y < 0 or y >= self.height:
                continue

            ## 9) Check if the point is in an obstacle
            if self.obstacle_map[y, x] == 1:
                self.failure_buffer.append(1)
                continue

            success = True
            self.failure_buffer.append(0)

        return Node(x, y)


    
    def nearest_node(self, rand_node):
        return min(self.tree, key=lambda node: (node.x - rand_node.x)**2 + (node.y - rand_node.y)**2)

    def steer(self, gen_node, near_nodes=None):
        # Get the node to steer towards

        # If near_nodes is None, we will get which nodes are near.
        to_node = gen_node
        if near_nodes is None:
            near_nodes = self.near(gen_node)
        
        # Find the best node to steer towards based on cost.
        best_node = None
        best_cost = float('inf')
        for node in near_nodes:
            if self.collision_free(node, gen_node):
                new_cost = node.node_cost + self.cost(node, gen_node)
                if new_cost < best_cost:
                    best_node = node
                    best_cost = new_cost
        
        # If no near nodes are found, use the nearest node in the tree..
        if best_node is None:
            best_node = self.nearest_node(gen_node)
        
        from_node = best_node
        
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        distance = self.cost(from_node, to_node)

        new_node = None
        if distance < self.step_size and self.collision_free(from_node, to_node):
            new_node = Node(to_node.x, to_node.y)
            new_node.parent = from_node
            new_node.node_cost = from_node.node_cost + distance
            return new_node
        else:            
            theta = np.arctan2(dy, dx)
            new_x = int(np.round(from_node.x + self.step_size * np.cos(theta)))
            new_y = int(np.round(from_node.y + self.step_size * np.sin(theta)))
            
            new_node = Node(new_x, new_y)
            new_node.parent = from_node
            new_node.node_cost = from_node.node_cost + self.cost(from_node, new_node)

            if self.collision_free(from_node, new_node) and self.obstacle_map[new_node.y, new_node.x] == 0:
                return new_node
            else:
                return None

    def collision_free(self, from_node, to_node):
        x0, y0 = int(from_node.x), int(from_node.y)
        x1, y1 = int(to_node.x), int(to_node.y)

        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy  # error value e_xy

        while True:
            if self.obstacle_map[y0, x0] == 1:
                return False
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return True
    
    ## Cost computation function
    ## Cost is the distance between two nodes
    def cost(self, node1, node2):
        return np.hypot(node1.x - node2.x, node1.y - node2.y)
    
    def near(self, new_node, radius=None):
        # Find all nodes within the radius of new_node
        near_nodes = []
        if radius is None:
            radius = self.step_size

        for node in self.tree:
            if self.cost(new_node,node) < radius:
                near_nodes.append(node)
        return near_nodes
    
    def is_goal_reached(self, node):
        return self.cost(self.goal,node) < self.goal_threshold

    def get_path(self, end_node):
        path = []
        node = end_node
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]  # reverse
    
    def rewire(self, new_node, near_nodes=None):
        if near_nodes is None:
            # Paper suggests rewiring within a radius of 2.5 * step_size
            near_nodes = self.near(new_node, self.step_size*2.5)

        # Rewire the tree by checking if any nearby nodes can be connected to the new node
        for node in near_nodes:
            if self.collision_free(new_node, node):
                new_cost = new_node.node_cost + self.cost(new_node, node)
                if new_cost < node.node_cost:
                    node.parent = new_node
                    node.node_cost = new_cost
    @timer
    def plan(self,video_output=None):
        video_frame_counter = 0
        iteration_counter = 0
        for iteration_counter in range(self.max_iter):
            rand_node = self.sample_random_point()
            near_nodes = self.near(rand_node)
            new_node = self.steer(rand_node, near_nodes)
            if new_node is None:
                continue
            
            if new_node is not None:
                
                self.tree.append(new_node)
                
                ## Rewire the tree
                self.rewire(new_node)

                if self.closest_node is None or self.closest_node_distance > self.cost(new_node, self.goal):
                    self.closest_node = new_node
                    self.closest_node_distance = self.cost(new_node, self.goal)
                    self.progress_buffer.append(1)
                    self.update_sigma()
                else:
                    self.progress_buffer.append(0)
                    self.update_sigma()

                video_frame_counter += 1
                if video_frame_counter == 1:
                    draw_map = self.visualize_map.copy()

                    # Code to draw sigma bounds.
                    ###                    
                    # Draw centerline bounds at ±sigma.
                    line_vec = np.array([self.goal.x - self.start.x, self.goal.y - self.start.y])
                    line_length = np.linalg.norm(line_vec)
                    if line_length == 0:
                        return  # Avoid division by zero.

                    line_unit = line_vec / line_length
                    line_normal = np.array([-line_unit[1], line_unit[0]])  # Perpendicular to centerline.

                    # Offset lines at ±sigma.
                    offset = self.sigma
                    start_point = np.array([self.start.x, self.start.y])
                    goal_point = np.array([self.goal.x, self.goal.y])

                    offset1_start = start_point + offset * line_normal
                    offset1_goal = goal_point + offset * line_normal
                    offset2_start = start_point - offset * line_normal
                    offset2_goal = goal_point - offset * line_normal

                    # Convert to integer pixel coordinates.
                    offset1_start = tuple(np.round(offset1_start).astype(int))
                    offset1_goal = tuple(np.round(offset1_goal).astype(int))
                    offset2_start = tuple(np.round(offset2_start).astype(int))
                    offset2_goal = tuple(np.round(offset2_goal).astype(int))

                    # Draw the two offset lines in gray.
                    cv2.line(draw_map, offset1_start, offset1_goal, color=(128, 128, 128), thickness=1)
                    cv2.line(draw_map, offset2_start, offset2_goal, color=(128, 128, 128), thickness=1)
                    ###

                    cv2.circle(draw_map, (rand_node.x,rand_node.y), 5, (255, 255, 255), -1)  # Mark the new node
                    
                    for node in self.tree:
                        if node.parent is not None:
                            cv2.line(draw_map, (node.parent.x, node.parent.y), (node.x, node.y), (255, 255, 255), 1)
                    video_output.write(draw_map)
                    video_frame_counter = 0
                    print(f"New Node: {new_node.x}, {new_node.y}")
                    # cv2.imshow("RRT", draw_map)
                    # cv2.waitKey(0)
                
                if self.is_goal_reached(new_node):
                    print(f"Goal Reached! with node count {len(self.tree)} within {iteration_counter} iterations")
                    return self.get_path(new_node),iteration_counter,len(self.tree)
            else:
                continue

        print("Failed to find a path")
        return None,self.max_iter,len(self.tree)

def create_video(filename="RRT_Traversal.mp4"):
    """
    Creates and initializes a VideoWriter object for saving the traversal video.

    This function sets up the video codec, filename, frame rate, and resolution for the output video.

    Returns:
        cv.VideoWriter: An OpenCV VideoWriter object for writing video frames.
    """
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 'mp4v' is commonly used for MP4 files
    
    # Define the frames per second (fps) for the video
    fps = 30
    # Initialize the VideoWriter object with the filename, codec, fps, and resolution
    video_output = cv2.VideoWriter(filename, fourcc, fps, (WIDTH, HEIGHT))
    return video_output  

def main():
    try:
        parameters = json.load(open("parameters.json"))
        start = (parameters["start"]['x'], parameters["start"]['y'])
        goal = (parameters["goal"]['x'], parameters["goal"]['y'])
        # clearance = int(np.ceil(float(parameters["clearance"]))/10) # in cm / pixels
        clearance = (parameters["clearance"]) 
        robot_radius = int(np.ceil(RRADIUS*100)) # in cm / pixels
        # clearance = int(np.ceil(float(input(f"Enter the clearance radius in mm: ")))/10) # in cm / pixels
        
        obstacle_map = gen_obstacle_map()
        # cv2.imshow("Obstacle Map", obstacle_map)
        expanded_map, expanded_mask = expand_obstacles(obstacle_map, robot_radius)
        # cv2.imshow("Expanded Map", expanded_map)
        expanded_map_2,expanded_mask_2 = expand_obstacles(expanded_map, clearance)
        # cv2.imshow("Expanded Map 2", expanded_mask_2)
        
        obstacle_map = np.where(expanded_mask_2 == 255, 1, 0).astype(np.uint8)
        # cv2.imshow("Expanded Map", expanded_map_2)
        # cv2.imshow("Obstacle Map 2", obstacle_map)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        cv2.circle(expanded_map_2, start , 5, (255, 0, 255), -1)
        cv2.circle(expanded_map_2, goal , 5, (255, 255, 0), -1)
        video_output = create_video(filename="RRT_star_N_v2_Traversal.mp4")
        cv2.putText(expanded_map_2, f"RRT*N", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        max_iterations,step_size,goal_sample_rate,goal_threshold = parameters['max_iterations'],parameters['step_size'],parameters['goal_sample_rate'],parameters['goal_threshold']
        rrt = RRT(start, goal, obstacle_map,expanded_map_2,max_iterations,step_size,goal_sample_rate,goal_threshold)
        path,iterations,node_count = rrt.plan(video_output=video_output)
        # path,iterations,node_count = rrt.plan()
        if path is not None:
            for i in range(len(path)-1):
                x, y = path[i]
                x2, y2 = path[i+1]
                # expanded_map_2[y, x] = [0, 255, 0]
                # Color the path green
                cv2.circle(expanded_map_2, (x, y), 3, (0, 255, 0), -1)
                cv2.line(expanded_map_2, (x, y), (x2, y2), (0, 255, 0), 1)
            
            for _ in range(30):
                # Write the frame to the video file
                cv2.putText(expanded_map_2, f"Iterations: {iterations} , Node count : {node_count}", (20, 290), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                video_output.write(expanded_map_2)
            
            # cv2.imshow("RRT Path", expanded_map_2)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
        else:
            print("No path found.")
            # video_output.release()
            cv2.putText(expanded_map_2, f"Iterations: {iterations} , Node count : {node_count}", (20, 290), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            # cv2.imshow("Obstacle Map", expanded_map_2)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
    finally:
        video_output.release()

if __name__ == "__main__":
    main()
