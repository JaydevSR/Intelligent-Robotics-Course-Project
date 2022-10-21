from sympy import Point, Polygon, Segment2D
import numpy as np
import matplotlib.pyplot as plt

#! Not working
def rrt_construction(start, goal, obstacles, arena_corner1, arena_corner2, num_nodes, max_length, neigh_rad):
    """
    Construct a Rapidly expanding Random Tree (RRT) for a given start and goal configuration
    """

    # Adjacency list representation of tree as a dictionary
    tree_forward = {goal: []}
    tree_backward = {goal: []}
    weights_forward = {goal: []}
    weights_backward = {goal: []}
    costs = {goal: 0}

    while len(tree_forward) != num_nodes:
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

            # add node
            tree_forward, tree_backward, weights_forward, weights_backward, costs, added_node = add_node_to_rrt(
                tree_forward, tree_backward,
                weights_forward, weights_backward,
                costs, rand_point, obstacles, max_length)

            # optimize
            tree_forward, tree_backward, weights_forward, weights_backward, costs = rewire_rrt(
                tree_forward, tree_backward,
                weights_forward, weights_backward,
                costs, added_node, obstacles, neigh_rad)
    
    # Add start and goal to tree
    # tree, weights = add_node_to_rrt(tree, weights, start, obstacles, max_length)
    # tree, weights = add_node_to_rrt(tree, weights, goal, obstacles, max_length)

    # get graph from forward and backward tree
    tree = {}
    weights = {}
    for r in tree_forward:
        tree[r] = tree_forward[r]
        weights[r] = weights_forward[r]
        for c in tree_forward[r]:
            if c in tree:
                tree[c].append(r)
                weights[c].append(r.distance(c))
            else:
                tree[c] = [r]
                weights[c] = [r.distance(c)]
    
    return tree, weights, costs

def add_node_to_rrt(tree_forward, tree_backward, weights_forward, weights_backward, costs, point, obstacles, max_length):
    # Find nearest neighbours
    distances = []
    keys = list(tree_forward.keys())
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
        added_node = point
        if dist > max_length:
            # generate an intermediate point
            theta = float(np.arctan2(float(point.y - nn.y), float(point.x - nn.x)))
            x = nn.x + max_length * np.cos(theta)
            y = nn.y + max_length * np.sin(theta)
            new_point = Point(x, y)
            added_node = new_point
            tree_forward[nn].append(new_point)
            weights_forward[nn].append(max_length)
            tree_backward[new_point] = [nn]
            tree_forward[new_point] = []
            weights_backward[new_point] = [max_length]
            weights_forward[new_point] = []
            costs[new_point] = costs[nn] + max_length
        else:
            tree_forward[nn].append(point)
            weights_forward[nn].append(dist)
            tree_backward[point] = [nn]
            tree_forward[point] = []
            weights_backward[point] = [nn]
            weights_forward[point] = [dist]
            costs[point] = costs[nn] + dist

    return tree_forward, tree_backward, weights_forward, weights_backward, costs, added_node

def rewire_rrt(tree_forward, tree_backward, weights_forward, weights_backward, costs, rand_point, obstacles, neigh_rad):
    # find neighbouring nodes
    neigh = set()
    for node in tree_forward:
        if node.distance(rand_point) < neigh_rad:
            neigh.add(node)

    for node in neigh:
        line = Segment2D(rand_point, node)
        flag = True
        for obs in obstacles:
            try:
                if not obs.intersect(line).is_empty:
                    flag = False
                    break
            except:
                print(obs)
                print(line)
                print(obs.intersect(line))
        if flag:
            new_cost = costs[rand_point] + line.length
            if new_cost < costs[node]:
                parent = tree_backward[node][0]
                # remove old edge
                idx = tree_forward[parent].index(node)
                tree_forward[parent].pop(idx)
                weights_forward[parent].pop(idx)
                # add new edge
                tree_forward[rand_point].append(node)
                weights_forward[rand_point].append(line.length)
                tree_backward[node] = [rand_point]
                weights_backward[node] = [line.length]
                costs[node] = new_cost
            
    return tree_forward, tree_backward, weights_forward, weights_backward, costs

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

    tree, weights, costs = rrt_construction(start, goal, obstacles, arena_corner1, arena_corner2, 20, 3, 6)

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
