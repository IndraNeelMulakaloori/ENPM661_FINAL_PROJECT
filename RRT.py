import numpy as np
import random
import math
import cv2
import json
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

    def sample_random_point(self):
        if random.random() < self.goal_sample_rate:
            return self.goal
        else:
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
            return Node(x, y)

    def nearest_node(self, rand_node):
        return min(self.tree, key=lambda node: (node.x - rand_node.x)**2 + (node.y - rand_node.y)**2)

    def steer(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        distance = math.hypot(dx, dy)
        if distance < self.step_size:
            return to_node
        theta = math.atan2(dy, dx)
        new_x = int(from_node.x + self.step_size * math.cos(theta))
        new_y = int(from_node.y + self.step_size * math.sin(theta))
        new_node = Node(new_x, new_y)
        new_node.parent = from_node
        return new_node

    def collision_free(self, from_node, to_node):
        # Bresenham's line or simple sampling
        steps = int(math.hypot(to_node.x - from_node.x, to_node.y - from_node.y))
        for i in range(steps):
            t = i / steps
            x = int(from_node.x * (1 - t) + to_node.x * t)
            y = int(from_node.y * (1 - t) + to_node.y * t)
            if self.obstacle_map[y, x] == 1:  # obstacle = 1 (or use your own convention)
                return False
        return True

    def is_goal_reached(self, node):
        return math.hypot(node.x - self.goal.x, node.y - self.goal.y) < self.goal_threshold

    def get_path(self, end_node):
        path = []
        node = end_node
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]  # reverse

    def plan(self,video_output=None):
        video_frame_counter = 0
        
        for iteration_counter in range(self.max_iter):
            rand_node = self.sample_random_point()
            nearest = self.nearest_node(rand_node)
            new_node = self.steer(nearest, rand_node)
            # cv2.circle(self.visualize_map, (rand_node.x,rand_node.y), 5, (255, 255, 255), -1)  # Mark the new node
            print(f"Nearest Node: {nearest.x}, {nearest.y}")
            print(f"New Node: {new_node.x}, {new_node.y}")
            if self.collision_free(nearest, new_node):
                cv2.line(self.visualize_map,(nearest.x,nearest.y),(new_node.x,new_node.y),(255,255,255),1)  # Mark the new node
                video_frame_counter += 1
                if video_frame_counter == 1:
                    # cv2.circle(self.visualize_map, (initial_position[1], initial_position[0]), 3, MAGENTA, -1)
                    # cv2.circle(self.visualize_map, (final_position[1], final_position[0]), 3, GREEN, -1)
                    video_output.write(self.visualize_map)
                    video_frame_counter = 0
                self.tree.append(new_node)
                if self.is_goal_reached(new_node):
                    print(f"Goal Reached! with node count {len(self.tree)} with {iteration_counter} iterations")
                    return self.get_path(new_node),iteration_counter,len(self.tree)

        print("Failed to find a path")
        return None,self.max_iter,len(self.tree)
def create_video(filename):
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
    cv2.circle(expanded_map_2, start , 5, (255, 255, 255), -1)
    cv2.circle(expanded_map_2, goal , 5, (255, 255, 0), -1)
    video_output = create_video(filename="RRT_Traversal.mp4")
    cv2.putText(expanded_map_2, f"RRT", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    max_iterations,step_size,goal_sample_rate,goal_threshold = parameters['max_iterations'],parameters['step_size'],parameters['goal_sample_rate'],parameters['goal_threshold']
    rrt = RRT(start, goal, obstacle_map,expanded_map_2,max_iterations,step_size,goal_sample_rate,goal_threshold)
    path,iterations,node_count = rrt.plan(video_output=video_output)
    
    if path is not None:
        for (x, y) in path:
            # expanded_map_2[y, x] = [0, 255, 0]
            # Color the path green
            cv2.circle(expanded_map_2, (x, y), 3, (0, 255, 0), -1)
    else:
        print("No path found.")
        # video_output.release()
    for _ in range(60):
            # Write the frame to the video file
            cv2.putText(expanded_map_2, f"Iterations: {iterations} , Node count : {node_count}", (20, 290), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            video_output.write(expanded_map_2)
        
        # video_output.release()
        
        
    cv2.imshow("RRT Path", expanded_map_2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # cv2.imshow("Expanded Map", expanded_map_2)
    # Release the video writer
    video_output.release()

if __name__ == "__main__":
    main()
