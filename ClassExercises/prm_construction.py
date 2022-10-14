from sympy import Point, Polygon, Segment2D
import numpy as np
import matplotlib.pyplot as plt

def prm_construction(start, goal, obstacles, arena_corner1, arena_corner2, num_nodes, n_q):
    """
    :param start: start point
    :param goal: goal point
    :param obstacles: list of obstacles (rectangular)
    :param arena: arena rectangle
    :param num_nodes: number of nodes in graph
    :param n_q: number of nearest neighbours
    :return: list of nodes and list of edges
    """

    # Adjacency list representation of graph as a dictionary
    graph = {}
    weights = {}

    while len(graph) != num_nodes:
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
            if len(graph) == 0: # first node
                graph[rand_point] = []
                weights[rand_point] = []
            else: # not first node
                graph, weights = add_node_to_graph(graph, weights, rand_point, obstacles, n_q)
    
    # Add start and goal to graph
    graph, weights = add_node_to_graph(graph, weights, start, obstacles, n_q)
    graph, weights = add_node_to_graph(graph, weights, goal, obstacles, n_q)

    return graph, weights


def add_node_to_graph(graph, weights, point, obstacles, n_q):
    # Find nearest neighbours
    distances = []
    keys = list(graph.keys())
    for node in keys:
        distances.append(point.distance(node))
    distances = np.array(distances)
    nearest_neighbours = distances.argsort()[:n_q]

    graph[point] = []
    weights[point] = []
    # Check if edge passes through obstacles
    for i in nearest_neighbours:
        flag = True
        line = Segment2D(point, keys[i])
        for o in obstacles:
            if not o.intersect(line).is_empty:
                flag = False
                break
        if flag:
            graph[point].append(keys[i])
            graph[keys[i]].append(point)
            distance = point.distance(keys[i])
            weights[point].append(distance)
            weights[keys[i]].append(distance)
        if len(graph) < n_q or len(graph[point]) == n_q:
            break
    if len(graph[point]) == 0:
        del graph[point]
        del weights[point]
    
    return graph, weights

    


if __name__ == "__main__":
    start = Point(1, 0)
    goal = Point(8, 10)
    # p1 = Polygon(Point(0, 1), Point(0, 2.5), Point(5, 2.5), Point(5, 1))
    # p2 = Polygon(Point(4, 4), Point(4, 7), Point(7, 7), Point(7, 4))
    # obstacles = [p1, p2]
    p = Polygon(Point(2, 2), Point(2, 5), Point(5, 5), Point(5, 2))
    obstacles = [p]

    arena_corner1 = Point(0, 0)
    arena_corner2 = Point(10, 10)

    graph, weights = prm_construction(start, goal, obstacles, arena_corner1, arena_corner2, 50, 3)

    # plot the graph
    # plot polygons
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect('equal')
    ax.set_xlim(-1, 11)
    ax.set_ylim(-1, 11)
    for p in obstacles:
        ax.add_artist(plt.Polygon([[v.x, v.y] for v in p.vertices], color='b'))

    # plot nodes and edges
    for node in graph:
        ax.plot(node.x, node.y, 'ko')
        for neighbour in graph[node]:
            ax.plot([node.x, neighbour.x], [node.y, neighbour.y], 'k-')
    
    # plot start and goal
    ax.plot(start.x, start.y, 'ro', markersize=10)
    ax.plot(goal.x, goal.y, 'go', markersize=10)

    plt.show()
