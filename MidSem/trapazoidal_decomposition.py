from sympy import Polygon, Point, Line, Segment2D, Point2D, Line2D

# Trapazoidal decomposition of multiple polygons
def decompose_into_trapazoids(arena, polygons):
    all_vertices = [v for p in polygons for v in p.vertices]
    all_vertices = list(set(all_vertices)) # remove duplicates
    all_vertices.sort(key=lambda v: v.args[0]) # sort by x

    lines = []
    current_sides = sp.FiniteSet()
    last_v = None
    for v in all_vertices:
        # skip vertices on the same x
        if not last_v is None and v.args[0] == last_v.args[0]:
            continue
        sweep_line = Line(Point(v.args[0], 0), Point(v.args[0], 1)) # vertical line
        intersections = sp.FiniteSet()
        for p in polygons:
            for s in p.sides:
                if not s.is_parallel(sweep_line):
                    intersections = intersections.union(s.intersect(sweep_line))
        intersections = intersections.union(arena.intersect(sweep_line))
        intersections = list(intersections)
        # print(intersections)
        last_v = v
        for i in range(0, len(intersections) - 1):
            line = Line(intersections[i], intersections[i+1])
            for p in polygons:
                isect = p.intersect(line)
                if not type(isect) is Segment2D:
                    mp = Point2D((line.args[0].args[0] + line.args[1].args[0]) / 2,
                                 (line.args[0].args[1] + line.args[1].args[1]) / 2)
                    if not p.encloses_point(mp) and list(isect)[0] in p.vertices:
                        lines.append(line)
    print(lines)
