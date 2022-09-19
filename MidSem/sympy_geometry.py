import sympy as sp

def get_line(x, y):
    return sp.Line2D(x, y)

def distance_between(x, y):
    return x.distance(y)

def distance_from_line(x, L):
    return L.distance(x)

def distance_from_polygon(x, P):
    return P.distance(x)

def tangents_to_polygon(x, P):
    return list(
        map(lambda s: sp.Line2D(x, s.args[0]),
            filter(lambda x : len(x) == 1,
                map(lambda v : P.intersect(sp.Line2D(x, v)),
                    P.vertices
                )
            )
        )
    )

def intersect_polygons(P1, P2):
    return P1.intersect(P2)
