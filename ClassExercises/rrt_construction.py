from sympy import Point, Polygon, Segment2D
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy

class Environment:
    def __init__(self, arena_corner1, arena_corner2, obstacles):
        self.corner1 = arena_corner1
        self.corner2 = arena_corner2
        self.obstacles = obstacles

class RRT:
    def __init__(self, root):
        self.root = root
        self.nodes = [root]
        self.costs = {root: 0}

class TreeNode:
    def __init__(self, point, parent):
        self.point = point
        self.parent = parent
        self.children = []
        self.c_weights = []

def rrt_construction(start, goal, env, num_nodes, max_length, optimize=False, neigh_rad=0):

    rrt = RRT(goal)

    while len(rrt.nodes) != num_nodes:
        # Generate random point in arena
        rand_point = Point(np.random.uniform(arena_corner1.x, arena_corner2.x),
                            np.random.uniform(arena_corner1.y, arena_corner2.y))

        flag = True
        for obstacle in env.obstacles:
            if obstacle.encloses_point(rand_point):
                flag = False

        # Connect to nearest neighbours
        if flag:
            rrt = add_node_to_rrt(rrt, rand_point, env, max_length)
            print(len(rrt.nodes), "Nodes")

    return rrt

def add_node_to_rrt(rrt, rand_point, env, max_length):
    # find nearest node
    nearest_node = rrt.nodes[0]
    dist = nearest_node.point.distance(rand_point)
    for node in rrt.nodes:
        node_dist = node.point.distance(rand_point)
        if node_dist < dist:
            nearest_node = node
            dist = node_dist

    flag = True
    line = Segment2D(nearest_node.point, rand_point)
    for obstacle in env.obstacles:
        if line.intersect(obstacle):
            flag = False

    # add node
    if flag and dist < max_length:
        new_node = TreeNode(rand_point, nearest_node)
        nearest_node.children.append(new_node)
        nearest_node.c_weights.append(dist)
        rrt.nodes.append(new_node)
        rrt.costs[new_node] = rrt.costs[nearest_node] + dist
    elif flag:
        new_point = nearest_node.point + (rand_point - nearest_node.point)*(max_length/dist)
        new_point = Point(float(new_point.x), float(new_point.y))
        new_node = TreeNode(new_point, nearest_node)
        nearest_node.children.append(new_node)
        nearest_node.c_weights.append(max_length)
        rrt.nodes.append(new_node)
        rrt.costs[new_node] = rrt.costs[nearest_node] + max_length

    return rrt

if __name__ == '__main__':
    start = TreeNode(Point(0, 0), None)
    goal = TreeNode(Point(40, 40), None)
    p = Polygon(Point(20, 20), Point(20, 40), Point(30, 40), Point(30, 20))
    obstacles = [p]

    arena_corner1 = Point(0, 0)
    arena_corner2 = Point(50, 50)

    env = Environment(arena_corner1, arena_corner2, obstacles)
    print("Environment created")

    print("Constructing RRT")
    rrt = rrt_construction(start, goal, env, 50, 5, optimize=True, neigh_rad=10)

    print("Plotting RRT")
    plot_rrt(rrt, env, start, goal)
    