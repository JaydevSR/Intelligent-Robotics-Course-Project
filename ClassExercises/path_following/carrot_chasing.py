from sympy import Point, Polygon, Segment2D
import numpy as np
import matplotlib.pyplot as plt

class Environment:
    def __init__(self, arena_corner1, arena_corner2, obstacles):
        self.corner1 = arena_corner1
        self.corner2 = arena_corner2
        self.obstacles = obstacles

class Vehicle:
    def __init__(self, location, heading, velocity, delta, umax, theta_gain, name=""):
        self.name = name
        self.location = location
        self.heading = heading
        self.velocity = velocity
        self.delta = delta
        self.umax = umax
        self.theta_gain = theta_gain

    def vehicle_info(self):
        print(f"Vehicle Info:")
        print(f"|   Name = {self.name}")
        print(f"|   Linear Velocity = {self.velocity}")
        print(f"|   Max Angular Change = {self.umax}")
        print(f"|   Heading Gain = {self.theta_gain}")
        print("\n\n")

    def vehicle_status(self):
        print(f"Status:")
        print(f"|   Location = ({float(self.location.x)}, {float(self.location.y)})")
        print(f"|   Heading = {self.heading}")
        print("\n\n")


def carrot_chasing(dt, vehicle, environment, path, goal_epsilon):
    planned_path = [vehicle.location]

    vehicle.vehicle_info()
    next_wp = np.argmin([vehicle.location.distance(wp) for wp in path])
    
    nn = 0
    while (path[-1].distance(vehicle.location) > goal_epsilon) and nn < 100:  # Main Loop
        nn += 1
        if vehicle.location.distance(path[next_wp+1]) < goal_epsilon:
            next_wp += 1
        
        vehicle.vehicle_status()
        print(f"Next Waypoint = {next_wp}\n")

        current_segment = Segment2D(path[next_wp], path[next_wp + 1])
        
        vehicle_x = float(vehicle.location.x)
        vehicle_y = float(vehicle.location.y)
        proj = current_segment.projection(vehicle.location)
        seg_theta = float(np.arctan(float(current_segment.slope)))
        carrot_x = float(proj.x + vehicle.delta * np.cos(seg_theta))
        carrot_y = float(proj.y + vehicle.delta * np.sin(seg_theta))

        desired_heading = np.arctan2((carrot_y - vehicle_y), (carrot_x - vehicle_x))

        u = vehicle.theta_gain * (desired_heading - vehicle.heading)
        u = clamp(u, -vehicle.umax, vehicle.umax)

        # integrate
        vehicle.heading = vehicle.heading + u*dt
        vehicle_x = vehicle_x + vehicle.velocity * np.cos(vehicle.heading) * dt
        vehicle_y = vehicle_y + vehicle.velocity * np.sin(vehicle.heading) * dt
        vehicle.location = Point(vehicle_x, vehicle_y)
        planned_path.append(vehicle.location)

    return planned_path

def clamp(x, x_min, x_max):
    if x > x_max:
        return x_max
    elif x < x_min:
        return x_min
    else:
        return x

if __name__ == "__main__":
    arena_corner1 = Point(0, 0)
    arena_corner2 = Point(50, 50)

    env = Environment(arena_corner1, arena_corner2, [])
    print("Environment created")

    # test path
    path = [Point(10, 10), Point(18, 22), Point(18, 30),  Point(30, 30), Point(40, 40)]

    # create vehicle
    dt = 1
    velocity = 1
    goal_epsilon = 1
    start_location = Point(10, 12)
    heading = np.pi / 4
    delta = 0.2
    umax = np.pi / 4
    theta_gain = 0.5
    my_car = Vehicle(start_location, heading, velocity, delta, umax, theta_gain)

    planned_path = carrot_chasing(0.5, my_car, env, path, goal_epsilon)

    # plot path
    x = [float(p.x) for p in path]
    y = [float(p.y) for p in path]
    plt.plot(x, y, 'bo-', label="Path")

    x = [float(p.x) for p in planned_path]
    y = [float(p.y) for p in planned_path]
    plt.plot(x, y, 'r', label="Planned Path")
    plt.plot(start_location.x, start_location.y, 'ro', label="Start")
    plt.plot(path[-1].x, path[-1].y, 'go', label="Goal (Path end)")
    plt.legend()
    plt.savefig("./carrot_chasing.png")
    plt.show()