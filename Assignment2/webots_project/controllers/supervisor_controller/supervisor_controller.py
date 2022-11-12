"""Supervisor controller."""

from controller import Supervisor
import math

def angle(robot_pos, goal_pos):
    x = robot_pos[0] - goal_pos[0]
    y = robot_pos[1] - goal_pos[1]
    rad = math.atan2(y, x)
    return rad + math.pi
    
def dist_betn(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 \
            + (p1[1] - p2[1]) ** 2)


robot = Supervisor()

############## Moving Obstacles ##############

ball_a = robot.getFromDef("BALL_A")
ball_b = robot.getFromDef("BALL_B")
ball_c = robot.getFromDef("BALL_C")

print(ball_a)

position_a = ball_a.getPosition()
position_b = ball_b.getPosition()
position_c = ball_c.getPosition()

tran_a = ball_a.getField('translation')
tran_b = ball_b.getField('translation')
tran_c = ball_c.getField('translation')

target_a = [-9, -2, 0.5]
target_b = [7, -2, 0.5]
target_c = [7, 3, 0.5]

velocity_a = dist_betn(target_a, position_a) / 1000
velocity_b = dist_betn(target_b, position_b) / 3000
velocity_c = dist_betn(target_c, position_c) / 3000


angle_a = angle(position_a, target_a)
angle_b = angle(position_b, target_b)
angle_c = angle(position_c, target_c)

timestep = 64

while robot.step(timestep) != -1:

    position_a = ball_a.getPosition()
    position_b = ball_b.getPosition()
    position_c = ball_c.getPosition()
    
    delta_pos_a = [velocity_a * math.cos(angle_a),
                   velocity_a * math.sin(angle_a), 0]
                                
    delta_pos_b = [velocity_b * math.cos(angle_b), 
                   velocity_b * math.sin(angle_b), 0]

    delta_pos_c = [velocity_c * math.cos(angle_c),
                     velocity_c * math.sin(angle_c), 0]

    if dist_betn(target_a, position_a) > 0.5:
        position_a = [position_a[i] + delta_pos_a[i] 
                        for i in range(3)]
        tran_a.setSFVec3f(position_a)
    else:
        tran_a.setSFVec3f(target_a)
    
    if dist_betn(target_b, position_b) > 0.5:
        position_b = [position_b[i] + delta_pos_b[i] 
                        for i in range(3)]
        tran_b.setSFVec3f(position_b)
    else:
        tran_b.setSFVec3f(target_b)

    if dist_betn(target_c, position_c) > 0.5:
        position_c = [position_c[i] + delta_pos_c[i] 
                        for i in range(3)]
        tran_c.setSFVec3f(position_c)
    else:
        tran_c.setSFVec3f(target_c)

############### Controller End ################
