import numpy as np
import cv2
import json


HEIGHT = 300  # cm
WIDTH = 600   # cm
RRADIUS = 0.22  # robot radius in meters


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.node_cost = 0
        self.parent = None


class MovingObstacle:
    def __init__(self, x_start, y_start, width=10, height=200, velocity=2):
        self.x = x_start
        self.y = y_start
        self.width = width
        self.height = height
        self.velocity = velocity

    def update(self):
        self.y += self.velocity
        if self.y + self.height > HEIGHT or self.y < 0:
            self.velocity *= -1

    def contains(self, x, y):
        return self.x <= x <= self.x + self.width and self.y <= y <= self.y + self.height


def gen_obstacle_map(moving_obstacle=None):
    map_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

    static_obs = [
        lambda x, y: (100 <= x <= 110) and (0 <= y <= 200),
        lambda x, y: (320 <= x <= 330) and (0 <= y <= 100),
        lambda x, y: (320 <= x <= 330) and (200 <= y <= 300),
        lambda x, y: (430 <= x <= 440) and (0 <= y <= 200)
    ]

    for y in range(HEIGHT):
        for x in range(WIDTH):
            if any(obs(x, y) for obs in static_obs) or (moving_obstacle and moving_obstacle.contains(x, y)):
                map_img[y, x] = (0, 0, 255)

    return np.flipud(map_img)


def expand_obstacles(image, radius):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 200, 200])
    upper_red = np.array([25, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
    dilated = cv2.dilate(mask, kernel)

    h, w = dilated.shape
    dilated[:radius + 1, :] = 255
    dilated[h - radius:, :] = 255

    output = image.copy()
    output[np.where(dilated == 255)] = [0, 165, 255]
    output[np.where(mask == 255)] = [0, 0, 255]

    return output, dilated


class RRT:
    def __init__(self, start, goal, obstacle_map):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.tree = [self.start]
        self.map = obstacle_map
        self.max_iter = 1000
        self.step_size = 30
        self.goal_sample_rate = 0.15
        self.goal_threshold = 5
        self.height, self.width = obstacle_map.shape

    def sample_random_point(self):
        if np.random.rand() < self.goal_sample_rate:
            return self.goal
        else:
            return self.probabilistic_node()

    def probabilistic_node(self):
        while True:
            line = np.array([self.goal.x - self.start.x, self.goal.y - self.start.y])
            norm = np.linalg.norm(line)
            unit = line / norm
            normal = np.array([-unit[1], unit[0]])
            point_on_line = np.array([self.start.x, self.start.y]) + np.random.rand() * line
            sigma = norm // 2
            offset = np.random.normal(0, sigma)
            point = point_on_line + offset * normal
            x, y = int(point[0]), int(point[1])
            if 0 <= x < self.width and 0 <= y < self.height and self.map[y, x] == 0:
                return Node(x, y)

    def nearest_node(self, rand_node):
        return min(self.tree, key=lambda n: (n.x - rand_node.x) ** 2 + (n.y - rand_node.y) ** 2)

    def steer(self, rand_node):
        near = self.nearest_node(rand_node)
        dx, dy = rand_node.x - near.x, rand_node.y - near.y
        dist = np.hypot(dx, dy)
        if dist == 0:
            return None
        scale = min(self.step_size, dist) / dist
        new_x = int(near.x + dx * scale)
        new_y = int(near.y + dy * scale)
        if 0 <= new_x < self.width and 0 <= new_y < self.height and self.map[new_y, new_x] == 0:
            new_node = Node(new_x, new_y)
            new_node.parent = near
            new_node.node_cost = near.node_cost + dist
            return new_node
        return None

    def is_goal_reached(self, node):
        return np.hypot(node.x - self.goal.x, node.y - self.goal.y) < self.goal_threshold

    def get_path(self, node):
        path = []
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]

    def plan(self):
        for i in range(self.max_iter):
            rand = self.sample_random_point()
            new_node = self.steer(rand)
            if new_node:
                self.tree.append(new_node)
                if self.is_goal_reached(new_node):
                    return self.get_path(new_node), i, len(self.tree)
        return None, self.max_iter, len(self.tree)


def create_video(filename):
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    return cv2.VideoWriter(filename, fourcc, 30, (WIDTH, HEIGHT))


def main_dynamic():
    parameters = json.load(open("../parameters.json"))
    start = (parameters["start"]['x'], parameters["start"]['y'])
    goal = (parameters["goal"]['x'], parameters["goal"]['y'])
    clearance = parameters["clearance"]
    robot_radius = int(np.ceil(RRADIUS * 100))

    video = create_video("../media/RRT_star_N_Dynamic_obstacles.mp4")
    moving_obstacle = MovingObstacle(210, 100)

    for step in range(100):
        print(f"\n--- Step {step} ---")
        moving_obstacle.update()
        raw_map = gen_obstacle_map(moving_obstacle)
        expanded1, mask1 = expand_obstacles(raw_map, robot_radius)
        expanded2, mask2 = expand_obstacles(expanded1, clearance)
        binary_map = (mask2 == 255).astype(np.uint8)

        frame = expanded2.copy()
        cv2.circle(frame, start, 5, (255, 0, 255), -1)
        cv2.circle(frame, goal, 5, (255, 255, 0), -1)

        rrt = RRT(start, goal, binary_map)
        path, iterations, node_count = rrt.plan()

        if path:
            for i in range(len(path) - 1):
                x1, y1 = path[i]
                x2, y2 = path[i + 1]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"Path found! Iter: {iterations}, Nodes: {node_count}", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        else:
            cv2.putText(frame, "No path found", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        video.write(frame)
        cv2.imshow("Dynamic RRT*N", frame)
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main_dynamic()
