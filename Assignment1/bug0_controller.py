"""Algorithm: Bug0"""

from controller import Robot, Motor, DistanceSensor
from helper_functions import distance_between, angle_of, on_line, distance_from_line

robot = Robot()
    
TIME_STEP = 64
MAX_SPEED = 6.28
GOAL_POSITION = [0.2, 0.7, 0.0]
POS_EPSILON = 0.05  # distance from goal when to stop
OBS_PROX = 150.0
ANGLE_EPSILON = 0.05

#initialize motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))  # number of radians the motor rotates
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# initialize devices
ps = []
ps_names = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]
for i in range(8):
    ps.append(robot.getDevice(ps_names[i]))
    ps[i].enable(TIME_STEP)

ps_values = [0 for i in range(8)]

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

state = 'begin'

while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    for i in range(8):
        ps_values[i] = ps[i].getValue()    
    current_position = gps.getValues()
    
    # initialize motor speeds at 50% of MAX_SPEED.
    left_speed  = 0.5 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED
    
    # at time begin
    if state == 'begin':
        start_position = gps.getValues()
        state = 'move_to_goal'

    elif state == 'move_to_goal':
        obstacle_detected = ps_values[0] > OBS_PROX or ps_values[7] > OBS_PROX
        if obstacle_detected:
            hit_point = gps.getValues()
            state = 'follow_obstacle'
        elif distance_between(current_position, GOAL_POSITION) <= POS_EPSILON:
            state = 'end'
        elif not on_line(current_position, start_position, GOAL_POSITION):
            # move back on line
            heading_angle = angle_of(start_position, current_position)
            goal_angle = angle_of(start_position, GOAL_POSITION)
            
            if (heading_angle - goal_angle) > ANGLE_EPSILON:
                print('turning right')
                left_speed  = 0.5 * MAX_SPEED
                right_speed = 0.1 * MAX_SPEED
            elif (heading_angle - goal_angle) < -ANGLE_EPSILON:
                print('turning left')
                left_speed  = 0.1 * MAX_SPEED
                right_speed = 0.5 * MAX_SPEED
        else:
            print('on line')
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            
    elif state == 'follow_obstacle':
        print('following obstacle')
        right_side_covered = ps_values[2] > OBS_PROX
        if not right_side_covered:
            left_speed  = -0.5 * MAX_SPEED
            right_speed = 0.5 * MAX_SPEED
        else:
            right_value = max(ps_values[0:2])
            left_value = max(ps_values[5:7])
            if right_value > 2.0*OBS_PROX:
                left_speed  = 0.20 * MAX_SPEED
                right_speed = 0.50 * MAX_SPEED
            elif right_value < OBS_PROX and left_value < OBS_PROX:
                left_speed  = 0.50 * MAX_SPEED
                right_speed = 0.20 * MAX_SPEED
            else:
                left_speed  = 0.50 * MAX_SPEED
                right_speed = 0.50 * MAX_SPEED
                
    elif state == 'end':
        print('reached end')
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        break
        
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
