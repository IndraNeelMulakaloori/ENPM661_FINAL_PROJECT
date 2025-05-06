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
import random
import math
import cv2
HEIGHT = 300 # cm
WIDTH = 600 # cm
WRADIUS = .033
RRADIUS = .22
WDIS = .287


        
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
    def __init__(self, start, goal, obstacle_map, visualize_map,max_iter=5000, step_size=5, goal_sample_rate=0.15, goal_threshold=5):
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

    def sample_random_point(self):
        ## Too high a goal_sample_rate (e.g. > 20%) might bias the search too much and ruin exploration.
        ## Recommended range: 5% to 15% for balance between exploration and goal convergence.
        random_sampling = np.random.rand()
        if random_sampling < self.goal_sample_rate:
            return self.goal
        else:
            return self.probability_density_function(random_sampling)

    def probability_density_function(self,random_sampling):
        ## 1) Compute the line vector from start to goal
        line_vec = np.array([self.goal.x - self.start.x, self.goal.y - self.start.y])
        ## 2) Normalize the line vector
        line_length = np.linalg.norm(line_vec)
        
        ## 3) Unit vector along the line
        line_unit = line_vec / line_length
        
        ## 4) Perpendicular vector to the line
        line_normal = np.array([-line_unit[1], line_unit[0]])

        ## 5) Randomly sample a point along the line
        # t = np.random.rand()
        point_on_line = np.array([self.start.x, self.start.y]) + random_sampling * line_vec

        ## 6) # Apply Gaussian offset perpendicular to the line
        sigma = line_length // 2  # Or dynamically adjust if needed
        offset_distance = np.random.normal(0, sigma)

        ## 7) Bias the point along the normal vector
        biased_point = point_on_line + offset_distance * line_normal
        x, y = int(biased_point[0]), int(biased_point[1])

        return Node(x, y)
    #### Dynamic sigma computation
    
    
    
    
    
    def nearest_node(self, rand_node):
        return min(self.tree, key=lambda node: (node.x - rand_node.x)**2 + (node.y - rand_node.y)**2)

    def steer(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        # distance = math.hypot(dx, dy)
        distance = self.cost(from_node, to_node)
        new_node = None
        if distance < self.step_size:
            new_node = Node(to_node.x, to_node.y)
            new_node.parent = from_node
            new_node.node_cost = from_node.node_cost + distance
            return new_node
        
        # theta_2 = math.atan2(dy, dx)
        theta = np.arctan2(dy, dx)
        new_x = int(from_node.x + self.step_size * np.cos(theta))
        new_y = int(from_node.y + self.step_size * np.sin(theta))
        
        new_node = Node(new_x, new_y)
        new_node.parent = from_node
        # new_node.node_cost = from_node.node_cost + self.step_size
        new_node.node_cost = from_node.node_cost + self.cost(from_node, new_node)
    
        # print(f"new_node cost: {new_node.node_cost}")
        # print(f"dup_node_cost: {dup_node_cost}")
        return new_node

    def collision_free(self, from_node, to_node):
        # Bresenham's line or simple sampling

        steps = int(np.hypot(to_node.x - from_node.x, to_node.y - from_node.y))
        for i in range(steps):
            t = i / steps
            x = int(from_node.x * (1 - t) + to_node.x * t)
            y = int(from_node.y * (1 - t) + to_node.y * t)
            if self.obstacle_map[y, x] == 1:  # obstacle = 1 (or use your own convention)
                return False
        return True
    ## Cost computation function
    ## Cost is the distance between two nodes
    def cost(self, node1, node2):
        return np.hypot(node1.x - node2.x, node1.y - node2.y)
    
    def near(self, new_node):
        # Find all nodes within the radius of new_node
        near_nodes = []
        tuning_constant = 35 # 30-50 cm is asweet spot tested (multiple times)
        n = len(self.tree)
        ## defining radius as per the RRT* paper
        ## r ∝ (log(n)/n)^(1/d) where n = number of nodes, d = dimension
        radius = tuning_constant * (np.log(n) / n)**(1/2) 
        # print(f"Radius: {radius}")
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

    def plan(self,video_output=None):
        video_frame_counter = 0
        iteration_counter = 0
        for iteration_counter in range(self.max_iter):
            rand_node = self.sample_random_point()
            nearest = self.nearest_node(rand_node)
            new_node = self.steer(nearest, rand_node)
            # cv2.circle(self.visualize_map, (rand_node.x,rand_node.y), 5, (255, 255, 255), -1)  # Mark the new node
            # print(f"Nearest Node: {nearest.x}, {nearest.y}")
            # print(f"New Node: {new_node.x}, {new_node.y}")
            if self.collision_free(nearest, new_node):
                cv2.line(self.visualize_map,(nearest.x,nearest.y),(new_node.x,new_node.y),(255,255,255),1)  # Mark the new node
                video_frame_counter += 1
                if video_frame_counter == 1:
                    # cv2.circle(self.visualize_map, (initial_position[1], initial_position[0]), 3, MAGENTA, -1)
                    # cv2.circle(self.visualize_map, (final_position[1], final_position[0]), 3, GREEN, -1)
                    video_output.write(self.visualize_map)
                    video_frame_counter = 0
                
                X_near_tree = self.near(new_node)
                x_min = nearest
                c_min = nearest.node_cost + self.cost(nearest, new_node)
                
                for node in X_near_tree:
                    if self.collision_free(node, new_node):
                        new_cost = node.node_cost + self.cost(node, new_node)
                        if new_cost < c_min:
                            x_min = node
                            c_min = new_cost

                new_node.parent = x_min
                new_node.node_cost = c_min
                self.tree.append(new_node)
                
                ## Rewire the tree
                for node in X_near_tree:
                    if self.collision_free(new_node, node):
                        new_cost = new_node.node_cost + self.cost(new_node, node)
                        if new_cost < node.node_cost:
                            node.parent = new_node
                            node.node_cost = new_cost
                
                if self.is_goal_reached(new_node):
                    print(f"Goal Reached! with node count {len(self.tree)} within {iteration_counter} iterations")
                    return self.get_path(new_node)

        print("Failed to find a path")
        return None
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
    start = (74, 50)
    goal = (510, 180)
    
    robot_radius = int(np.ceil(RRADIUS*100)) # in cm / pixels
    # clearance = int(np.ceil(float(input(f"Enter the clearance radius in mm: ")))/10) # in cm / pixels
    clearance = 5 # in cm / pixels
    
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
    cv2.circle(expanded_map_2, start , 5, (255, 255, 255), -1)
    cv2.circle(expanded_map_2, goal , 5, (255, 255, 0), -1)
    video_output = create_video(filename="RRT_star_N_Traversal.mp4")
    rrt = RRT(start, goal, obstacle_map,expanded_map_2)
    path = rrt.plan(video_output=video_output)
    # path = rrt.plan()
    
    if path is not None:
        for (x, y) in path:
            # expanded_map_2[y, x] = [0, 255, 0]
            # Color the path green
            cv2.circle(expanded_map_2, (x, y), 3, (0, 255, 0), -1)
        
        for _ in range(30):
            # Write the frame to the video file
            video_output.write(expanded_map_2)
        
        # video_output.release()
        
        
        cv2.imshow("RRT Path", expanded_map_2)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("No path found.")
        # video_output.release()
        cv2.imshow("Obstacle Map", expanded_map_2)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    # cv2.imshow("Expanded Map", expanded_map_2)
    # Release the video writer
    video_output.release()

if __name__ == "__main__":
    main()
