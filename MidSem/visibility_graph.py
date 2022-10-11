from sympy import Polygon, Point, Line, Segment2D, Point2D, Line2D
from shortest_path import *
import matplotlib.pyplot as plt

# visibility graph of multiple polygons
def visibility_graph(start, goal, polygons):
    points = [start, goal]
    for p in polygons:
        points.extend(p.vertices)
    points = list(set(points)) # remove duplicates
    points.sort(key=lambda v: v.args[0]) # sort by x

    edges = []
    for p in polygons:
        edges.extend(p.sides)

    visgraph = []

    for p in range(len(points)):
        for q in range(len(points)):
            if p == q:
                continue
            line = Segment2D(points[p], points[q])
            flag = True
            for e in edges:
                isect = e.intersect(line)
                if isect.is_empty:
                    continue
                elif type(isect) is Segment2D:
                    continue
                elif not list(isect)[0] in e.args:
                    flag = False
                    break
            for pol in polygons:
                # calculate midpoint of line
                if pol.encloses_point(line.midpoint):
                    flag = False
                    break
            if flag:
                visgraph.append((points[p], points[q]))
    return visgraph

if __name__ == "__main__":
    # environment
    start = Point(1, 0)
    goal = Point(8, 10)
    p1 = Polygon(Point(0, 1), Point(0, 2.5), Point(5, 2.5), Point(5, 1))
    p2 = Polygon(Point(4, 4), Point(4, 7), Point(7, 7), Point(7, 4))
    polygons = [p1, p2]

    # plot polygons
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(10, 10))
    for p in polygons:
        ax1.add_patch(plt.Polygon(p.vertices, closed=True, fill=True))
        ax2.add_patch(plt.Polygon(p.vertices, closed=True, fill=True))
        ax3.add_patch(plt.Polygon(p.vertices, closed=True, fill=True))
        ax4.add_patch(plt.Polygon(p.vertices, closed=True, fill=True))

    visgraph = visibility_graph(start, goal, polygons)
    # print("Visibility graph:")
    # for e in visgraph:
    #     print(e)
    
    # plot visibility graph
    for e in visgraph:
        ax2.plot([e[0].args[0], e[1].args[0]], [e[0].args[1], e[1].args[1]], 'b-')
    ax2.plot([], [], 'b-', label="Visibility Graph")

    adj_list, weights = get_adjacency_list(visgraph)
    path = a_star(start, goal, adj_list, weights)
    # plot shortest path
    for i in range(len(path)-1):
        ax4.plot([path[i].args[0], path[i+1].args[0]], [path[i].args[1], path[i+1].args[1]], 'r-')
    ax4.plot([], [], 'r-', label="Shortest Path")
    # print("Shortest path:")
    # print(path)

    # Bug 0 path
    bug0_path = [
        start,
        Point2D(17/10, 1),
        Point(0, 1),
        Point(0, 2.5),
        Point2D(4, 25/4),
        Point2D(4, 7),
        goal
    ]

    # plot Bug 0 path
    for i in range(len(bug0_path)-1):
        ax3.plot([bug0_path[i].args[0], bug0_path[i+1].args[0]], [bug0_path[i].args[1], bug0_path[i+1].args[1]], 'g-')
        ax4.plot([bug0_path[i].args[0], bug0_path[i+1].args[0]], [bug0_path[i].args[1], bug0_path[i+1].args[1]], 'g-')
    ax3.plot([], [], 'g-', label="Bug 0 Path")
    ax4.plot([], [], 'g-', label="Bug 0 Path")

    # plot start and goal
    for ax in [ax1, ax2, ax3, ax4]:
        ax.axis('equal')
        ax.plot(start.args[0], start.args[1], 'ro', label='start', markersize=10)
        ax.plot(goal.args[0], goal.args[1], 'go', label='goal', markersize=10)
        # set limits
        ax.set_xlim(-1, 11)
        ax.set_ylim(-1, 11)

    # legend
    for ax in [ax2, ax3, ax4]:
        ax.legend()

    # plot title
    plt.title("Visibility graph, Shortest path and Bug 0 path")

    # save plot
    plt.savefig("MidSem/plots/visibility_graph.png")


    plt.show()
