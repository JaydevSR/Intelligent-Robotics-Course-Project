from sympy import Polygon, Point, Line, Segment2D, Point2D, Line2D

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

    for p in range(len(points)-1):
        for q in range(p+1, len(points)):
            line = Line(points[p], points[q])
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
            if flag:
                visgraph.append((points[p], points[q]))
    return set(visgraph)
