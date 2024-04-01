import math
import heapq
import matplotlib.pyplot as plt

class GridMap:
    def __init__(self, obstacles, start, goal):
        self.obstacles = obstacles
        self.start = start
        self.goal = goal
        self.costs = {}
        # Definition of possible movement directions (up, down, left, right)
        self.movements = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    def distance(self, point1, point2):
        # Function to calculate the Euclidean distance between two points
        return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

    def calculate_theta(self, point1, point2):
        delta_x = point2[0] - point1[0]
        delta_y = point2[1] - point1[1]
        return math.atan2(delta_y, delta_x)

    def a_star(self, movement_cost=lambda point1, point2: 1):#uniform cost
        priority_queue = []
        heapq.heappush(priority_queue, (0, self.start))
        costs = {self.start: 0}
        parents = {}

        while priority_queue:
            cost_current, node_current = heapq.heappop(priority_queue)
            if node_current == self.goal:
                path = []
                while node_current in parents:
                    path.append(node_current)
                    node_current = parents[node_current]
                path.append(self.start)
                path.reverse()
                return path

            for dx, dy in self.movements:
                neighbor = (node_current[0] + dx, node_current[1] + dy)
                new_cost = costs[node_current] + movement_cost(node_current, neighbor)
                if neighbor not in costs or new_cost < costs[neighbor]:
                    if neighbor not in self.obstacles:
                        costs[neighbor] = new_cost
                        priority = new_cost + self.distance(neighbor, self.goal)
                        heapq.heappush(priority_queue, (priority, neighbor))
                        parents[neighbor] = node_current

        return None

    def normalize_angle(self, theta):
        return (theta + math.pi) % (2 * math.pi) - math.pi

    def plot_map(self, path=None):
        plt.figure(figsize=(8, 8))

        for x, y in self.costs:
            plt.text(x, y, str(self.costs[(x, y)]), ha='center', va='center', fontsize=8, color='gray')
            if (x, y) in self.obstacles:
                plt.plot(x, y, 'ks', markersize=10)

        if path:
            path_x = [point[0] for point in path]
            path_y = [point[1] for point in path]
            plt.plot(path_x, path_y, 'r-', linewidth=2)

        plt.plot(self.start[0], self.start[1], 'go', markersize=10)
        plt.plot(self.goal[0], self.goal[1], 'bo', markersize=10)

        plt.grid(True)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Grid Map and Path with Costs')
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

# Example usage:
obstacles = [(1, 1), (2, 2), (3, 3)]  # Define obstacles
start = (0, 0)
goal = (4, 4)

grid_map = GridMap(obstacles, start, goal)
for x in range(5):
    for y in range(5):
        grid_map.costs[(x, y)] = x + y

path = grid_map.a_star()
if path:
    # Calculate angles for each point in the path
    for i in range(len(path)-1):
        current_point = path[i]
        next_point = path[i+1]
        theta = grid_map.calculate_theta(current_point, next_point)
        theta = grid_map.normalize_angle(theta)  # Normalize angle between -pi and pi
        print("Coordinates (x, y, theta) =", current_point[0], current_point[1], math.degrees(theta))
else:
    print("No path found")

grid_map.plot_map(path)