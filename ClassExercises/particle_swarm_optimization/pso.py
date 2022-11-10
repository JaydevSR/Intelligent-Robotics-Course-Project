import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt

def distance(p1, p2):
    return norm(p1 - p2)

def get_random_path(lower, upper, dim=2, npoints=4):
    return np.random.uniform(lower, upper, size=dim*npoints).reshape(npoints, dim)

def get_random_velocities(scaling, dim=2, npoints=4):
    return scaling*(np.random.rand(npoints, dim) - 0.5)

def get_path_cost(source, goal, path):
    cost = 0
    npoints, dim = path.shape
    for i in range(npoints):
        cost += distance(path[i], path[i-1])
    cost += distance(source, path[0])
    cost += distance(goal, path[-1])
    return cost

def particle_swarm_optimization(source, goal, env_side,
                                n_particles=10,
                                n_waypoints=4, 
                                n_iterations=100,
                                max_velocity=1,
                                local_gain=1,
                                global_gain=2,
                                velocity_weight=0.5):
    
    # generate random initial configuration
    particle_paths = {}
    particle_velocities = {}
    particle_costs = {}
    global_best_cost = np.Inf
    local_best_cost = np.Inf
    local_best_path = None
    global_best_path = None
    global_best_history = []
    for i in range(n_particles):
        particle_paths[i] = get_random_path(0, env_side, dim=2, npoints=n_waypoints)
        particle_velocities[i] = get_random_velocities(max_velocity, dim=2, npoints=n_waypoints)
        particle_costs[i] = get_path_cost(source, goal, particle_paths[i])
        if particle_costs[i] < local_best_cost:
            local_best_cost = particle_costs[i]
            local_best_path = particle_paths[i]
    
    global_best_path = local_best_path
    global_best_cost = local_best_cost
    global_best_history.append(global_best_cost)

    for t in range(n_iterations):
        local_best_cost = np.Inf
        local_best_path = None
        print(f"Iteration: {t}")
        # Update positions
        for i in range(n_particles):
            particle_paths[i] = particle_paths[i] + particle_velocities[i]
            particle_costs[i] = get_path_cost(source, goal, particle_paths[i])
            if particle_costs[i] < local_best_cost:
                local_best_cost = particle_costs[i]
                local_best_path = particle_paths[i]

        # Update velocities
        for i in range(n_particles):
            particle_velocities[i] = velocity_weight * particle_velocities[i] + \
                local_gain * np.random.rand() * (local_best_path - particle_paths[i]) + \
                global_gain * np.random.rand() * (global_best_path - particle_paths[i])

        # Update global best
        if local_best_cost < global_best_cost:
            global_best_cost = local_best_cost
            global_best_path = local_best_path
    
        global_best_history.append(global_best_cost)
        ##############################
    
    return global_best_path, global_best_cost, global_best_history


if __name__ == "__main__":
    # np.random.seed(0)
    source = np.array([5, 5])
    goal = np.array([15, 15])
    env_side = 20
    n_particles = 40
    n_waypoints = 4
    n_iterations = 100
    max_velocity = 1
    local_gain = 1
    global_gain = 2
    velocity_weight = 0.5

    path, cost, cost_history = particle_swarm_optimization(source, goal, env_side,
                                             n_particles=n_particles,
                                             n_waypoints=n_waypoints,
                                             n_iterations=n_iterations,
                                             max_velocity=max_velocity,
                                             local_gain=local_gain,
                                             global_gain=global_gain,
                                             velocity_weight=velocity_weight)

    print("Source-Goal distance: ", distance(source, goal))
    # print("Path: ", path)
    print("Cost: ", cost)
    # print("Cost history: ", cost_history)

    plt.plot(cost_history)
    plt.show()

    
    # plot complete path
    plt.plot([source[0], path[0, 0]], [source[1], path[0, 1]], 'b')
    for i in range(n_waypoints-1):
        plt.plot([path[i, 0], path[i+1, 0]], [path[i, 1], path[i+1, 1]], 'b')
    plt.plot([path[-1, 0], goal[0]], [path[-1, 1], goal[1]], 'b')

    plt.plot(source[0], source[1], 'ro')
    plt.plot(goal[0], goal[1], 'go')
    plt.show()
