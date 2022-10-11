from sympy import Point2D, Circle, Line, Polygon, N
import numpy as np
import matplotlib.pyplot as plt

def potential_field_planner(start, goal, obstacles, goal_threshold, obstacle_threshods, repulsive_gain, attractive_gain, step_size, eps):
    path = [start]
    position = start
    forces = get_forces(position, goal, obstacles, goal_threshold, obstacle_threshods, repulsive_gain, attractive_gain)
    while forces[0] > eps and forces[1] > eps:
        position = gradient_descent_step(position, forces, step_size)
        path.append(position)
        forces = get_forces(position, goal, obstacles, goal_threshold, obstacle_threshods, repulsive_gain, attractive_gain)
        print(float(position.x), float(position.y))
    return path

def gradient_descent_step(position, forces, step_size):
    new_position = [0, 0]
    new_position[0] = float(position.x + step_size * forces[0])
    new_position[1] = float(position.y + step_size * forces[1])
    return Point2D(new_position)

def get_forces(position, goal, obstacles, goal_threshold, obstacle_threshods, repulsive_gain, attractive_gain):
    force = [0, 0]
    for i in range(len(obstacles)):
        obstacle = obstacles[i]
        if type(obstacle) is Circle:
            dist = position.distance(obstacle.center) - obstacle.radius
            if dist < obstacle_threshods[i]:
                force[0] += float(repulsive_gain * (1/obstacle_threshods[i] - 1/dist) * (position.x - obstacle.center.x) / (dist + obstacle.radius) / dist**2)
                force[1] += float(repulsive_gain * (1/obstacle_threshods[i] - 1/dist) * (position.y - obstacle.center.y) / (dist + obstacle.radius) / dist**2)
        else: 
            # error
            raise ValueError("Obstacle type not supported")
    
    dist = goal.distance(position)
    # Calculate the attractive force
    if dist < goal_threshold: # quadratic potential function
        force[0] += float(attractive_gain * (goal.x - position.x))
        force[1] += float(attractive_gain * (goal.y - position.y))
    else: # linear potential function
        force[0] += float(attractive_gain * goal_threshold * (goal.x - position.x) / dist)
        force[1] += float(attractive_gain * goal_threshold * (goal.y - position.y) / dist)
    return force

def path_length(path):
    length = 0
    for i in range(1, len(path)):
        length += float(path[i].distance(path[i-1]))
    return length

if __name__ == "__main__":
    start = Point2D(0, 0)

    # environment
    goal = Point2D(8, 7)
    goal_threshold = 5
    obstacles = [Circle(Point2D(5, 6), 1),
                Circle(Point2D(3, 8), 1),
                Circle(Point2D(2, 3), 0.5),
                Circle(Point2D(5, 2), 1)]
    obstacle_threshods = [2, 2, 2, 2]

    # paramerters
    repulsive_gain = -1
    attractive_gain = 2
    step_size = 0.02
    eps = 0.01
    
    path = potential_field_planner(start, goal, obstacles, goal_threshold, obstacle_threshods, repulsive_gain, attractive_gain, step_size, eps)
    
    print("Path length: ", path_length(path))

    # visualize
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim(-1, 11)
    ax.set_ylim(-1, 11)
    ax.plot([p[0] for p in path], [p[1] for p in path], 'r')
    # plot obstacles
    for obstacle in obstacles:
        if type(obstacle) is Circle:
            ax.add_artist(plt.Circle((obstacle.center.x, obstacle.center.y), obstacle.radius, color='b'))
        else:
            ax.add_artist(plt.Polygon([[v.x, v.y] for v in obstacle.vertices], color='b'))
    ax.plot(goal.x, goal.y, 'go')
    ax.plot(start[0], start[1], 'ro')
    plt.show()

