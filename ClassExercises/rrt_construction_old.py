from sympy import Point, Polygon, Segment2D
import numpy as np
import matplotlib.pyplot as plt

def rrt_construction(start, goal, obstacles, arena_corner1, arena_corner2, num_nodes, max_length):
    """
    Construct a Rapidly expanding Random Tree (RRT) for a given start and goal configuration
    """

    # Adjacency list representation of tree as a dictionary
    tree = {goal: []}
    weights = {goal: []}

    while len(tree) != num_nodes:
        # Generate random point in arena
        rand_point = Point(np.random.uniform(arena_corner1.x, arena_corner2.x),
                            np.random.uniform(arena_corner1.y, arena_corner2.y))

        flag = True
        # Check if point is in obstacles
        for o in obstacles:
            if o.encloses_point(rand_point):
                flag = False

        # Connect to nearest neighbours
        if flag:
            # if len(tree) == 0: # first node
            #     tree[rand_point] = []
            #     weights[rand_point] = []
            # else: # not first node
            tree, weights = add_node_to_rrt(tree, weights, rand_point, obstacles, max_length)
    
    # Add start and goal to tree
    # tree, weights = add_node_to_rrt(tree, weights, start, obstacles, max_length)
    # tree, weights = add_node_to_rrt(tree, weights, goal, obstacles, max_length)

    return tree, weights

def add_node_to_rrt(tree, weights, point, obstacles, max_length):
    # Find nearest neighbours
    distances = []
    keys = list(tree.keys())
    for node in keys:
        distances.append(point.distance(node))
    distances = np.array(distances)
    nn_idx = distances.argsort()[0]


    nn = keys[nn_idx]
    flag = True
    line = Segment2D(point, nn)
    for o in obstacles:
        if not o.intersect(line).is_empty:
            flag = False
            break
    
    if flag: # no obstacle in the way
        dist = line.length
        if dist > max_length:
            # generate an intermediate point
            theta = float(np.arctan2(float(point.y - nn.y), float(point.x - nn.x)))
            x = nn.x + max_length * np.cos(theta)
            y = nn.y + max_length * np.sin(theta)
            new_point = Point(x, y)
            tree[new_point] = [nn]
            weights[new_point] = [max_length]
            tree[nn].append(new_point)
            weights[nn].append(max_length)
        else:
            tree[point] = [nn]
            weights[point] = [dist]
            tree[nn].append(point)
            weights[nn].append(dist)

    return tree, weights

if __name__ == "__main__":
    start = Point(0, 0)
    goal = Point(40, 40)
    # p1 = Polygon(Point(0, 1), Point(0, 2.5), Point(5, 2.5), Point(5, 1))
    # p2 = Polygon(Point(4, 4), Point(4, 7), Point(7, 7), Point(7, 4))
    # obstacles = [p1, p2]
    p = Polygon(Point(20, 20), Point(20, 40), Point(30, 40), Point(30, 20))
    obstacles = [p]

    arena_corner1 = Point(0, 0)
    arena_corner2 = Point(50, 50)

    tree, weights = rrt_construction(start, goal, obstacles, arena_corner1, arena_corner2, 100, 5)

    # plot the tree
    # plot polygons
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')
    ax.set_xlim(-10, 60)
    ax.set_ylim(-10, 60)
    for p in obstacles:
        ax.add_artist(plt.Polygon([[v.x, v.y] for v in p.vertices], color='b'))

    # plot nodes and edges
    for node in tree:
        ax.plot(node.x, node.y, 'ko')
        for neighbour in tree[node]:
            ax.plot([node.x, neighbour.x], [node.y, neighbour.y], 'k-')
    
    # plot start and goal
    ax.plot(start.x, start.y, 'ro', markersize=10)
    ax.plot(goal.x, goal.y, 'go', markersize=10)

    plt.show()
