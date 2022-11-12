from controller import Robot, Motor, DistanceSensor
import math

############# Helper Functions ################
def get_bearing_in_degrees(north):
    rad = math.atan2(north[0], north[1])
    bearing = (rad - 1.5708) / 3.14 * 180.0
    bearing += 180
    if bearing < 0.0:
        bearing = bearing + 360.0
    return bearing
    
def angle_of(A, B):
    return math.degrees(math.atan2((B[1]-A[1]), (B[0]-A[0])) % 360) + 90 

def distance_between(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

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

def clamp(x, min, max):
    if x < min:
        return min
    elif x > max:
        return max
    else:
        return x
        
############ Controller Begin ##############
        
robot = Robot()

# Constants    
TIME_STEP = 64
MAX_SPEED = 15.0
GOAL_POSITION = [7.0, 7.0, 0.0]
POS_EPSILON = 0.4  # distance from goal when to stop

#initialize motors
left_motor = robot.getDevice('left wheel')
right_motor = robot.getDevice('right wheel')
left_motor.setPosition(float('inf'))  # number of radians the motor rotates
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# initialize devices
sense_left = robot.getDevice('sense_left')
sense_right = robot.getDevice('sense_right')
sense_left.enable(TIME_STEP)
sense_right.enable(TIME_STEP)

sense_front = robot.getDevice('sense_front')
sense_front.enable(TIME_STEP)

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

compass = robot.getDevice('compass')
compass.enable(TIME_STEP)

state = 'moving'

current_position = gps.getValues()
goal_distance_start = distance_between(current_position, GOAL_POSITION)


while robot.step(TIME_STEP) != -1 and state != 'reached_goal':
    # read sensors outputs
    left_value = 20.24 * sense_left.getValue()**(-4.76) + 0.6632
    right_value = 20.24 * sense_right.getValue()**(-4.76) + 0.6632
    front_value = 20.24 * sense_front.getValue()**(-4.76) + 0.6632    

    current_position = gps.getValues()
    current_angle = get_bearing_in_degrees(compass.getValues())
    
    # initialize motor speeds at 50% of MAX_SPEED.
    left_speed  = 0.5 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED
    
    goal_distance = distance_between(current_position, GOAL_POSITION)
    goal_angle = angle_of(current_position, GOAL_POSITION)
    angle_diff = goal_angle - current_angle

    # proportional control to turn towards goal
    left_speed -= 0.5 * MAX_SPEED * angle_diff / 180.0
    right_speed += 0.5 * MAX_SPEED * angle_diff / 180.0
    
    # closest bstacle to the right
    if right_value < 3.0 and right_value < left_value and right_value < front_value:
        left_speed -= 0.25 * MAX_SPEED * (3.0 - right_value) / 3.0
        right_speed += 0.25 * MAX_SPEED * (3.0 - right_value) / 3.0

    # closest obstacle to the left
    if left_value < 3.0 and left_value < right_value and left_value < front_value:
        left_speed += 0.25 * MAX_SPEED * (3.0 - left_value) / 3.0
        right_speed -= 0.25 * MAX_SPEED * (3.0 - right_value) / 3.0

    # if obstacle in front, turn away from it
    if front_value < 3.0:
        if left_value > right_value:
            left_speed = -0.5 * MAX_SPEED
            right_speed = 0.5 * MAX_SPEED
        else:
            left_speed = 0.5 * MAX_SPEED
            right_speed = -0.5 * MAX_SPEED
    
    # clamp the speed to the maximum allowed speed
    left_speed = clamp(left_speed, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed, -MAX_SPEED, MAX_SPEED)

    print(left_speed / MAX_SPEED)
    print(right_speed / MAX_SPEED)
    # If we are close to the goal, stop
    if distance_between(current_position, GOAL_POSITION) < POS_EPSILON:
        state = 'reached_goal'
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
    else:
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

############ Controller End ##############
