from sympy import Point, Polygon, Segment2D
import numpy as np
import matplotlib.pyplot as plt

class Vehicle:
    def __init__(self, dt, location, velocity, vmax, kp,
                ki=0, windup_steps=5, name="", follow=None, follow_distance=0):
        self.name = name
        self.location = location
        self.velocity = velocity
        self.vmax = vmax
        self.acceleration = 0
        self.follow = follow
        self.follow_distance = follow_distance
        self.kp = kp
        self.ki = ki
        self.integral = 0
        self.windup_steps = windup_steps
        self.integral_steps = 0
        self.dt = dt
        self.error = 0
        self.history = {"location": [], "velocity": [], "error": [], "acceleration": []}

    def PIcontrol(self):
        if self.follow is not None:
            curr_distance = (self.follow.location - self.location)
            self.error = curr_distance - self.follow_distance

            # TODO: Implement Integral control
            self.acceleration = self.kp * self.error

    def update(self):
        self.velocity += self.acceleration * self.dt
        self.velocity = clamp(self.velocity, 0, self.vmax)
        self.location += self.velocity * self.dt
        self.history["location"].append(self.location)
        self.history["velocity"].append(self.velocity)
        self.history["error"].append(self.error)
        self.history["acceleration"].append(self.acceleration)
        return self.location

    def vehicle_info(self):
        print(f"Vehicle Info:")
        print(f"|   Name = {self.name}")
        print(f"|   Time step = {self.dt}")
        print(f"|   Linear Velocity = {self.velocity}")
        print(f"|   Max Velocity = {self.umax}")
        print(f"|   Proportional Gain = {self.kp}")
        print(f"|   Integral Gain = {self.ki}")
        print("\n\n")

    def vehicle_status(self):
        print(f"Status:")
        print(f"|   Current location = {float(self.location)}")
        print(f"|   Current velocity = {self.velocity}")
        print(f"|   Current error = {self.error}")
        print("\n\n")

def clamp(x, x_min, x_max):
    if x > x_max:
        return x_max
    elif x < x_min:
        return x_min
    else:
        return x

if __name__=="__main__":
    # Create the vehicle
    dt = 0.1
    location1 = -10
    location2 = 10
    follow_distance = 5
    velocity2 = 1
    vmax = 2
    kp = 100
    ki = 0
    vehicle2 = Vehicle(dt, location2, velocity2, vmax, kp, ki, name="Vehicle 2")
    vehicle1 = Vehicle(dt, location1, 0, vmax, kp, ki, name="Vehicle 1", follow=vehicle2, follow_distance=follow_distance)

    total_time = 30
    time = np.arange(0, total_time, dt)
    for tstep in time:
        print(f"Time step: {tstep}\n")
        vehicle1.PIcontrol()
        vehicle2.PIcontrol()
        vehicle1.update()
        vehicle2.update()

        vehicle1.vehicle_status()
        vehicle2.vehicle_status()

    # plot the results
    plt.figure()
    plt.plot(time, vehicle1.history["location"], label="Vehicle 1")
    plt.plot(time, vehicle2.history["location"], label="Vehicle 2")
    plt.xlabel("Time (s)")
    plt.ylabel("Location (m)")
    plt.legend()
    plt.show()

    # plot error
    plt.figure()
    plt.plot(time, vehicle1.history["error"], label="Vehicle 1")
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.legend()
    plt.show()
