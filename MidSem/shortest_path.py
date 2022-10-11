from sympy import Point, Line, Segment2D, Point2D, Line2D

# get weight of edge
def get_weight(e):
    return e[0].distance(e[1])

# Get adjacency list from edges
def get_adjacency_list(edges):
    adj_list = {}
    weights = {}
    for e in edges:
        if e[0] not in adj_list:
            adj_list[e[0]] = []
            weights[e[0]] = []
        if e[1] not in adj_list:
            adj_list[e[1]] = []
            weights[e[1]] = []
        weight = get_weight(e)
        adj_list[e[0]].append(e[1])
        weights[e[0]].append(weight)
        adj_list[e[1]].append(e[0])
        weights[e[1]].append(weight)
    return adj_list, weights

# Dijkstra's algorithm
def dijkstra(start, goal, adj_list, weights):
    # initialize
    dist = {}
    prev = {}
    for v in adj_list:
        dist[v] = float('inf')
        prev[v] = None
    dist[start] = 0

    # main loop
    Q = set(adj_list.keys())
    while Q:
        u = min(Q, key=lambda v: dist[v])
        Q.remove(u)
        if u == goal:
            break
        for v in adj_list[u]:
            alt = dist[u] + weights[u][adj_list[u].index(v)]
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u

    # reconstruct path
    path = []
    u = goal
    while u:
        path.append(u)
        u = prev[u]
    path.reverse()
    return path


def a_star(start, goal, adj_list, weights):
    # initialize
    dist = {}
    prev = {}
    for v in adj_list:
        dist[v] = float('inf')
        prev[v] = None
    dist[start] = 0

    # main loop
    Q = set(adj_list.keys())
    while Q:
        u = min(Q, key=lambda v: dist[v] + v.distance(goal))
        Q.remove(u)
        if u == goal:
            break
        for v in adj_list[u]:
            alt = dist[u] + weights[u][adj_list[u].index(v)]
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u

    # reconstruct path
    path = []
    u = goal
    while u:
        path.append(u)
        u = prev[u]
    path.reverse()
    return path
