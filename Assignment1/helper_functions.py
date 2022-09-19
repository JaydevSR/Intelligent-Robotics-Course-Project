"""Helper functions for Bug simulations"""
import math

# Coordinate Geometry
def get_line(A, B):
    m = (B[1] - B[0]) / (A[1] - A[0])
    c = B[0] - m*A[0]
    return m, c

def distance_between(A, B):
    return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

def angle_of(A, B):
    return math.degrees(math.atan2((B[1]-A[1]), (B[0]-A[0])) % 360) + 90 
    
def distance_from_line(x, A, B):
    length_AB = distance_between(A, B)
    distance_from_AB = abs((B[1]-A[1])*(A[0] - x[0]) - (B[0]-A[0])*(A[1]-x[1]))
    distance_from_AB /= length_AB
    return distance_from_AB
    
def distance_from_polygon(x, pairs):
    distances = []
    for (A, B) in pairs:
        distances.append(
            distance_from_line(x, A, B)
        )
    return min(distances)


# For Bug algorithms
def on_line(x, A, B, tolerance=0.02):   
    dist = distance_from_line(x, A, B)
    if dist > tolerance:
        return False
    else:
        return True

def get_bearing_in_degrees(north):
    rad = math.atan2(north[0], north[1])
    bearing = (rad - 1.5708) / 3.14 * 180.0
    bearing += 180
    if bearing < 0.0:
        bearing = bearing + 360.0

    return bearing
