import sys
sys.path.append("..")
import PathPlanning.utils as utils
from PathPlanning.planner import Planner

class PlannerAStar(Planner):
    def __init__(self, m, inter=10):
        super().__init__(m)
        self.inter = inter
        self.initialize()

    def initialize(self):
        self.queue = []
        self.parent = {}
        self.open_set = []  # open set
        self.closed_set = []  # closed set
        self.start = {} # Distance from start to node
        self.goal = {} # Distance from node to goal
        self.f = {}  # Total cost
        self.goal_node = None
        self.img = None

    def get_neighbors(self, node, inter):
        neighbors = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                new_node = (node[0] + (i * inter), node[1] + (j * inter))
                if (
                    0 <= new_node[0] <= self.img.shape[1]
                    and 0 <= new_node[1] <= self.img.shape[0]
                    and self.map[new_node[1], new_node[0]] == 1
                ):
                    neighbors.append(new_node)
        return neighbors
    
    def planning(self, start=(100,200), goal=(375,520), inter=None, img=None):
        if inter is None:
            inter = self.inter
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        # Initialize 
        self.initialize()
        self.open_set.append(start)
        self.parent[start] = None
        self.start[start] = utils.distance(start, goal)
        self.goal[start] = 0
        self.f[start] = self.goal[start] + self.start[start]
        self.img = img
        while (1):
            # TODO: A Star Algorithm
            current_node = min(self.open_set, key=lambda x: self.f[x])
            if current_node == goal:
                self.goal_node = current_node
                break

            self.open_set.remove(current_node)
            self.closed_set.append(current_node)

            neighbors = self.get_neighbors(current_node, inter)

            for neighbor in neighbors:
                if neighbor in self.closed_set:
                    continue

                tenteative_g = self.goal[current_node] + \
                    utils.distance(current_node, neighbor)

                if neighbor not in self.open_set or tenteative_g < self.goal[neighbor]:
                    self.parent[neighbor] = current_node
                    self.start[neighbor] = utils.distance(neighbor, goal)
                    self.goal[neighbor] = tenteative_g
                    self.f[neighbor] = self.start[neighbor] + self.goal[neighbor]
                    if neighbor not in self.open_set:
                        self.open_set.append(neighbor)
        
        # Extract path
        path = []
        p = self.goal_node
        if p is None:
            return path
        while(True):
            path.insert(0,p)
            if self.parent[p] is None:
                break
            p = self.parent[p]
        if path[-1] != goal:
            path.append(goal)
        return path
