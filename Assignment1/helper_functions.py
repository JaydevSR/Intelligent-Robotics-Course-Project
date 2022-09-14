"""Helper functions for Bug simulations"""
import numpy as np
import math

def distance_between(A, B):
    return np.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

def angle_of(A, B):
    y = (A[1] - B[1])
    x = (A[0] - B[0])
    return math.atan2(y, x)
    
def distance_from_line(x, A, B):
    length_AB = distance_between(A, B)
    distance_from_AB = abs((B[1]-A[1])*(A[0] - x[0]) - (B[0]-A[0])*(A[1]-x[1]))
    distance_from_AB /= length_AB
    return distance_from_AB
    
def on_line(x, A, B, tolerance=0.02):      
    dist = distance_from_line(x, A, B)
    if dist > tolerance:
        return False
    else:
        return True
