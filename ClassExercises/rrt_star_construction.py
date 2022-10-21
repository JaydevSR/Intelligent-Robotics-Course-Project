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
    if optimize:
        rrt_star = RRT(deepcopy(goal))

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
            rrt, added_node = add_node_to_rrt(rrt, rand_point, env, max_length)
            print(len(rrt.nodes), "Nodes")
            if optimize and added_node is not None:
                star_node = deepcopy(added_node)
                rrt_star.nodes.append(star_node)
                rrt_star.costs[star_node] = rrt.costs[added_node]
                print("|   Optimizing...")
                rrt_star = rewire_rrt(rrt_star, star_node, env, neigh_rad)

    return rrt, rrt_star

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
    added_node = None
    if flag and dist < max_length:
        new_node = TreeNode(rand_point, nearest_node)
        nearest_node.children.append(new_node)
        nearest_node.c_weights.append(dist)
        rrt.nodes.append(new_node)
        rrt.costs[new_node] = rrt.costs[nearest_node] + dist
        added_node = new_node
    elif flag:
        new_point = nearest_node.point + (rand_point - nearest_node.point)*(max_length/dist)
        new_point = Point(float(new_point.x), float(new_point.y))
        new_node = TreeNode(new_point, nearest_node)
        nearest_node.children.append(new_node)
        nearest_node.c_weights.append(max_length)
        rrt.nodes.append(new_node)
        rrt.costs[new_node] = rrt.costs[nearest_node] + max_length
        added_node = new_node

    return rrt, added_node

def rewire_rrt(rrt, center_point, env, neigh_rad):
    # find neighbours
    neighbours = []
    for node in rrt.nodes:
        if node.point.distance(center_point.point) < neigh_rad:
            neighbours.append(node)

    # check if any of the neighbours can be rewired
    for node in neighbours:
        if node.point == center_point.point:
            continue
        flag = True
        line = Segment2D(center_point.point, node.point)
        for obstacle in env.obstacles:
            if line.intersect(obstacle):
                flag = False
        
        new_dist = center_point.point.distance(node.point)
        if flag and rrt.costs[node] > rrt.costs[center_point] + new_dist:
            # remove node from parent
            idx = node.parent.children.index(node)
            node.parent.children.pop(idx)
            node.parent.c_weights.pop(idx)

            node.parent = center_point
            center_point.children.append(node)
            center_point.c_weights.append(new_dist)
            rrt.costs[node] = rrt.costs[center_point] + new_dist

    return rrt

def plot_rrt(rrt, env, start, goal):
    fig, ax = plt.subplots()
    # plot root
    plt.plot(rrt.root.point.x, rrt.root.point.y, 'bo')

    for node in rrt.nodes:
        # plot node
        plt.plot(node.point.x, node.point.y, 'bo')
        if node.parent is not None:
            plt.plot([node.point.x, node.parent.point.x], [node.point.y, node.parent.point.y], 'k-')

    for p in env.obstacles:
        ax.add_artist(plt.Polygon([[v.x, v.y] for v in p.vertices], color='b', alpha=0.5))

    plt.plot(start.point.x, start.point.y, 'go')
    plt.plot(goal.point.x, goal.point.y, 'ro')

    plt.show()

def plot_rrt_star(rrt, rrt_star, env, start, goal):
    fig, (ax, ax_star) = plt.subplots(1, 2)
    ax.set_title("RRT")
    ax_star.set_title("RRT*")

    # plot root
    ax.plot(rrt.root.point.x, rrt.root.point.y, 'bo')
    ax_star.plot(rrt_star.root.point.x, rrt_star.root.point.y, 'bo')

    for node in rrt.nodes:
        # plot node
        ax.plot(node.point.x, node.point.y, 'bo')
        if node.parent is not None:
            ax.plot([node.point.x, node.parent.point.x], [node.point.y, node.parent.point.y], 'k-')

    for node in rrt_star.nodes:
        # plot node
        ax_star.plot(node.point.x, node.point.y, 'bo')
        if node.parent is not None:
            ax_star.plot([node.point.x, node.parent.point.x], [node.point.y, node.parent.point.y], 'k-')

    for p in env.obstacles:
        ax.add_artist(plt.Polygon([[v.x, v.y] for v in p.vertices], color='b', alpha=0.5))
        ax_star.add_artist(plt.Polygon([[v.x, v.y] for v in p.vertices], color='b', alpha=0.5))

    ax.plot(start.point.x, start.point.y, 'go')
    ax.plot(goal.point.x, goal.point.y, 'ro')
    ax_star.plot(start.point.x, start.point.y, 'go')
    ax_star.plot(goal.point.x, goal.point.y, 'ro')

    plt.show()

    return fig

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
    rrt, rrt_star = rrt_construction(start, goal, env, 200, 2, optimize=True, neigh_rad=5)

    print("Plotting RRT")
    # plot_rrt(rrt, env, start, goal)
    # plot_rrt(rrt_star, env, start, goal)
    fig = plot_rrt_star(rrt, rrt_star, env, start, goal)
    fig.savefig('rrt_star.png')
