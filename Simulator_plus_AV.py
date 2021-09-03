#!/usr/bin/python3

#from controller import Robot, Supervisor

from controller import Robot, Supervisor
from math import sqrt
import time
import math
import threading
import redis
import sys
import importlib


sys.path.append("..")
import get_path as g
importlib.reload(g)

enable_av = 0 #0 disabled, 1 Enabled communication with AV

if enable_av == 1:
    r = redis.Redis("192.168.1.156",6379)
    #r = redis.Redis("192.168.1.156",6379, charset="utf-8", decode_responses=True)
#print((r.get('set')).decode('utf-8'))

TIME_STEP = 64

robot = Supervisor()
robot_node = robot.getFromDef("MY_ROBOT")
if robot_node is None:
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)
trans_field = robot_node.getField("translation")

#robot = Robot()
ds = []
dsNames = ['ds_right']

#For Lidar
lidar = robot.getDevice('LDS-01')
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

lidar_main_motor = robot.getDevice('LDS-01_main_motor')
lidar_secondary_motor = robot.getDevice('LDS-01_secondary_motor')
lidar_main_motor.setPosition(float('+inf'))
lidar_secondary_motor.setPosition(float('+inf'))
lidar_main_motor.setVelocity(15.0)
lidar_secondary_motor.setVelocity(20.0)

#For Camera
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

#For Compass
compass = robot.getDevice('compass')
compass.enable(TIME_STEP)

#Enable Distance sensors
for i in range(1):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)

#Enable Wheels
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

avoidObstacleCounter = 0


wheelspeed_init = 6.9 #forward speed
turnspeed_init = 4
turncount_init = 33

def read_compass():
    compass_val = compass.getValues()
    #print(compass_val)
    rad = math.atan2(compass_val[0], compass_val[2])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0;
    #print(int(bearing))
    return int(bearing)

def go_straight(distance):#obs + human
    global wheelspeed_init
    wheelspeed = wheelspeed_init
    #wheelspeed = 3.2
    dist_init = 0
    exit_fwd = 0 #Flag to exit function after travelling distance
    fwd = 0 #To set forward only once
    dist = 0
    obstacle_count = 0
    #r.set("speed","1")
    
    while robot.step(TIME_STEP) != -1 and exit_fwd == 0:
        #read_compass()
        if enable_av == 1:
            r.set("speed","1")
            if (r.get('obs')).decode('utf-8') == 'true':
                #stopping Real robot
                if fwd == 1:
                    if (r.get('robot')).decode('utf-8') != 'stop':
                        print("Robot Stopping: Obstacle found")
                        r.set("robot", "stop")
                    #Stopping Simulator robot
                    wheelspeed = 0
                    wheels[0].setVelocity(wheelspeed)
                    wheels[1].setVelocity(wheelspeed)
                    fwd = 0

                if (distance - dist) >= 1.5:
                    if (r.get('human')).decode('utf-8') == '1':
                        obstacle_count = 0
                        continue # Keep waiting until human is present
                    else:
                        obstacle_count += 1

                    if obstacle_count >= 20:
                        time.sleep(1)
                        #go_around_right() #Go around if an non human obstacle is found
                        dist += 1.5
                        obstacle_count = 0
                continue #Keep looping until obstacle is removed
            else:
                wheelspeed = wheelspeed_init
                #wheelspeed = 3.2
                #fwd = 0
        #Calculate distance travelled by simulator robot
        values = trans_field.getSFVec3f()
        if dist_init == 0:
            x1 = values[0]
            z1 = values[2]
            dist_init = 1
        else:
            x2 = values[0]
            z2 = values[2]
            dist = sqrt((x2-x1)**2 + (z2-z1)**2)
            #print("Dist:", dist)
            if dist >= distance-0.01:
                exit_fwd = 1
                wheelspeed = 0
                if enable_av == 1:
                    print("Robot: Stopping")
                    r.set("robot","stop")
        
        # if dist<0.3 or dist>distance-0.3:
            # r.set("speed","0")
            # wheelspeed = 3.2
        # else:
            # r.set("speed","1")
            # wheelspeed = wheelspeed_init
        if enable_av == 1 and fwd == 0:
            r.set("robot","stop")
            r.set("robot", "forward")
            print("going straight")
            fwd = 1

        leftSpeed =  wheelspeed
        rightSpeed =  wheelspeed
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)

def go_reverse(distance):#obs + human
    global wheelspeed_init
    wheelspeed = wheelspeed_init
    #wheelspeed = 3.2
    dist_init = 0
    exit_fwd = 0 #Flag to exit function after travelling distance
    fwd = 0 #To set forward only once
    dist = 0
    obstacle_count = 0
    r.set("speed","1")
    
    while robot.step(TIME_STEP) != -1 and exit_fwd == 0:
        #read_compass()
        if enable_av == 1:
            if (r.get('obs')).decode('utf-8') == 'true':
                #stopping Real robot
                if fwd == 1:
                    if (r.get('robot')).decode('utf-8') != 'stop':
                        print("Robot Stopping: Obstacle found")
                        r.set("robot", "stop")
                    #Stopping Simulator robot
                    wheelspeed = 0
                    wheels[0].setVelocity(wheelspeed)
                    wheels[1].setVelocity(wheelspeed)
                    fwd = 0

                if (distance - dist) >= 1.5:
                    if (r.get('human')).decode('utf-8') == '1':
                        obstacle_count = 0
                        continue # Keep waiting until human is present
                    else:
                        obstacle_count += 1

                    if obstacle_count >= 20:
                        time.sleep(1)
                        #go_around_right() #Go around if an non human obstacle is found
                        dist += 1.5
                        obstacle_count = 0
                continue #Keep looping until obstacle is removed
            else:
                wheelspeed = wheelspeed_init
                #wheelspeed = 3.2
                #fwd = 0
        #Calculate distance travelled by simulator robot
        values = trans_field.getSFVec3f()
        if dist_init == 0:
            x1 = values[0]
            z1 = values[2]
            dist_init = 1
        else:
            x2 = values[0]
            z2 = values[2]
            dist = sqrt((x2-x1)**2 + (z2-z1)**2)
            #print("Dist:", dist)
            if dist >= distance-0.01:
                exit_fwd = 1
                wheelspeed = 0
                if enable_av == 1:
                    print("Robot: Stopping")
                    r.set("robot","stop")
        
        # if dist<0.3 or dist>distance-0.3:
            # r.set("speed","0")
            # wheelspeed = 3.2
        # else:
            # r.set("speed","1")
            # wheelspeed = wheelspeed_init
        if enable_av == 1 and fwd == 0:
            r.set("robot","stop")
            r.set("robot", "reverse")
            print("going reverse")
            fwd = 1

        leftSpeed =  -wheelspeed
        rightSpeed =  -wheelspeed
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)


def go_straight_obs(distance): # only obs detection
    global wheelspeed_init
    wheelspeed = wheelspeed_init
    dist_init = 0
    exit_fwd = 0
    fwd = 0
    dist = 0
    while robot.step(TIME_STEP) != -1 and exit_fwd == 0:
        read_compass()
        if enable_av == 1:
            if (r.get('obs')).decode('utf-8') == 'true':
                if fwd == 1:
                    if (r.get('robot')).decode('utf-8') != 'stop':
                        print("Robot Stopping: Obstacle found")
                        r.set("robot", "stop")
                    #Stopping Simulator robot
                    wheelspeed = 0
                    wheels[0].setVelocity(wheelspeed)
                    wheels[1].setVelocity(wheelspeed)
                    fwd = 0
                continue
            else:
                wheelspeed = wheelspeed_init
                #fwd = 0
        values = trans_field.getSFVec3f()
        if dist_init == 0:
            x1 = values[0]
            z1 = values[2]
            dist_init = 1
        else:
            x2 = values[0]
            z2 = values[2]
            dist = sqrt((x2-x1)**2 + (z2-z1)**2)
            #print("Dist:", dist)
            if dist >= distance-0.01:
                exit_fwd = 1
                wheelspeed = 0
                if enable_av == 1:
                    print("Robot: Stopping")
                    r.set("robot","stop")
                #quit()

        if enable_av == 1 and fwd == 0:
            r.set("robot","stop")
            r.set("robot", "forward")
            print("going straight")
            fwd = 1


        leftSpeed =  wheelspeed
        rightSpeed =  wheelspeed
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)

def turn_left():
    global turnspeed_init, turncount_init
    wheelspeed = turnspeed_init
    turn_count= turncount_init +3 #32 is good
    lef = 0
    #r.set("speed","0")
    while robot.step(TIME_STEP) != -1 and turn_count > 0:
        turn_count -= 1

        if turn_count<=0:
            wheelspeed = 0
            if enable_av == 1:
                time.sleep(.1)
                r.set("robot","stop")
                r.set("speed","1")

        if enable_av == 1 and lef == 0:
            print("taking left")
            r.set("speed","0")
            r.set("robot","stop")
            r.set("robot","left")
            lef = 1

        leftSpeed = -wheelspeed
        rightSpeed = wheelspeed
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)

def turn_right():
    global turnspeed_init, turncount_init
    wheelspeed = turnspeed_init
    turn_count= turncount_init
    rig = 0
    #r.set("speed","0")
    while robot.step(TIME_STEP) != -1 and turn_count > 0:
        turn_count -= 1

        if turn_count<=0:
            wheelspeed = 0
            if enable_av == 1:
                #time.sleep(.1)
                r.set("robot","stop")
                r.set("speed","1")

        if enable_av == 1 and rig == 0:
            print("taking right")
            r.set("speed","0")
            r.set("robot","stop")
            r.set("robot","right")
            rig = 1
        leftSpeed = wheelspeed
        rightSpeed = -wheelspeed
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)

def spin_anticlockwise360():
    global turnspeed_init, turncount_init
    wheelspeed = turnspeed_init
    turn_count= 64
    lef = 0
    r.set("speed","1")
    while robot.step(TIME_STEP) != -1 and turn_count > 0:
        turn_count -= 1

        if turn_count<=0:
            wheelspeed = 0
            if enable_av == 1:
                time.sleep(.1)
                r.set("robot","stop")
                r.set("speed","1")

        if enable_av == 1 and lef == 0:
            print("taking left")
            r.set("robot","stop")
            r.set("robot","left")
            lef = 1

        leftSpeed = -wheelspeed
        rightSpeed = wheelspeed
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)
        
def spin_clockwise360():
    global turnspeed_init, turncount_init
    wheelspeed = turnspeed_init
    turn_count= 66
    rig = 0
    r.set("speed","1")
    while robot.step(TIME_STEP) != -1 and turn_count > 0:
        turn_count -= 1

        if turn_count<=0:
            wheelspeed = 0
            if enable_av == 1:
                #time.sleep(.1)
                r.set("robot","stop")
                r.set("speed","1")

        if enable_av == 1 and rig == 0:
            print("taking right")
            r.set("robot","stop")
            r.set("robot","right")
            rig = 1
        leftSpeed = wheelspeed
        rightSpeed = -wheelspeed
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)


        
def turn_left_org():
    global wheelspeed_init
    wheelspeed = wheelspeed_init
    turn_count= 17
    lef = 0
    while robot.step(TIME_STEP) != -1 and turn_count > 0:
        turn_count -= 1

        if turn_count<=0:
            wheelspeed = 0
            if enable_av == 1:
                r.set("robot","stop")

        if enable_av == 1 and lef == 0:
            print("taking left")
            r.set("robot","stop")
            r.set("robot","left")
            lef = 1

        leftSpeed = -wheelspeed
        rightSpeed = wheelspeed
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)

def turn_right_org():
    global wheelspeed_init
    wheelspeed = wheelspeed_init
    turn_count= 17
    rig = 0
    while robot.step(TIME_STEP) != -1 and turn_count > 0:
        turn_count -= 1

        if turn_count<=0:
            wheelspeed = 0
            if enable_av == 1:
                r.set("robot","stop")

        if enable_av == 1 and rig == 0:
            print("taking right")
            r.set("robot","stop")
            r.set("robot","right")
            rig = 1
        leftSpeed = wheelspeed
        rightSpeed = -wheelspeed
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)

def turn_right_compass():
    global wheelspeed_init
    wheelspeed = wheelspeed_init
    initial_direction = read_compass()

    while robot.step(TIME_STEP) != -1:
        current_direction = read_compass()
        diff = initial_direction - current_direction
        if diff>180:
            delta = 360 - abs(diff)
        elif diff < -180:
            delta = -(360-abs(diff))
        else:
            delta = -diff
        print('Delta:', delta)

        if delta<=80:
            wheelspeed = wheelspeed_init
        elif delta<90:
            wheelspeed = 1
        else:
            wheelspeed = 0
            leftSpeed =  wheelspeed
            rightSpeed =  wheelspeed
            wheels[0].setVelocity(leftSpeed)
            wheels[1].setVelocity(rightSpeed)
            break

        leftSpeed = wheelspeed
        rightSpeed = -wheelspeed
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)

def turn(direction, angle=90):
    global wheelspeed_init
    wheelspeed = wheelspeed_init
    initial_direction = read_compass()
    if direction.lower() == 'left' or direction.lower() == 'right':
        if direction.lower() == 'left':
            angle = -angle
    else:
        print("Incorrect turn command")
        return


    while robot.step(TIME_STEP) != -1:
        current_direction = read_compass()
        diff = initial_direction - current_direction
        if diff>180:
            delta = 360 - abs(diff)
        elif diff < -180:
            delta = -(360-abs(diff))
        else:
            delta = -diff
        print('Delta:', delta)

        if direction.lower() == 'right':
            if delta <= angle-10:
                wheelspeed = wheelspeed_init
            elif delta < angle:
                wheelspeed = 1
            else:
                wheelspeed = 0
                leftSpeed =  wheelspeed
                rightSpeed =  wheelspeed
                wheels[0].setVelocity(leftSpeed)
                wheels[1].setVelocity(rightSpeed)
                break
            leftSpeed = wheelspeed
            rightSpeed = -wheelspeed
            wheels[0].setVelocity(leftSpeed)
            wheels[1].setVelocity(rightSpeed)
        else:
            if delta >= angle+10:
                wheelspeed = wheelspeed_init
            elif delta > angle:
                wheelspeed = 1
            else:
                wheelspeed = 0
                leftSpeed =  wheelspeed
                rightSpeed =  wheelspeed
                wheels[0].setVelocity(leftSpeed)
                wheels[1].setVelocity(rightSpeed)
                break

            leftSpeed = -wheelspeed
            rightSpeed = wheelspeed
            wheels[0].setVelocity(leftSpeed)
            wheels[1].setVelocity(rightSpeed)

def stop():
    if enable_av == 1:
        print("Robot: Stopping")
        r.set("robot","stop")
    wheelspeed = 0
    leftSpeed =  wheelspeed
    rightSpeed =  wheelspeed
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)

def atob():
    go_straight(1.5)
    time.sleep(1)
    turn_left()
    time.sleep(1)
    go_straight(6)
    time.sleep(1)
    turn_left()
    time.sleep(1)
    go_straight(2)
    time.sleep(1)
    turn_right()
    time.sleep(1)
    go_straight(2)
    time.sleep(1)
    turn_right()
    time.sleep(1)
    go_straight(2)
    time.sleep(1)

def btoa():
    #turn_right()
    #time.sleep(1)
    go_straight(8)
    time.sleep(1)
    turn_right()
    time.sleep(1)
    go_straight(1.5)
    time.sleep(1)
    turn_right()
    time.sleep(1)
    turn_right()
    time.sleep(1)

def atob_new():
    go_straight(4)#ha
    time.sleep(1)
    turn_left()
    time.sleep(1)
    go_straight(4)#ae
    time.sleep(1)
    turn_left()
    time.sleep(1)
    go_straight(2)#eb
    time.sleep(1)
    turn_right()
    time.sleep(1)
    go_straight(2)#bc
    time.sleep(1)
    turn_right()
    time.sleep(1)
    go_straight(2)#cd
    time.sleep(1)


# def atob_new():
    # go_straight(7)#ha
    # time.sleep(1)
    # turn_left()
    # time.sleep(1)
    # go_straight(2.5)#ae
    # time.sleep(1)
    # turn_left()
    # time.sleep(1)
    # go_straight(3)#eb
    # time.sleep(1)
    # turn_right()
    # time.sleep(1)
    # go_straight(4)#bc
    # time.sleep(1)
    # turn_right()
    # time.sleep(1)
    # go_straight(3)#cd
    # time.sleep(1)

def atob_rev():
    turn_right()
    time.sleep(1)
    turn_right()
    time.sleep(1)
    go_straight(3.6)#ha
    time.sleep(1)
    turn_left()
    time.sleep(1)
    go_straight(3.5)#ae
    time.sleep(1)
    turn_left()
    time.sleep(1)
    go_straight(3.6)#eb
    time.sleep(1)
    turn_right()
    time.sleep(1)
    go_straight(2.5)#bc
    time.sleep(1)
    turn_right()
    time.sleep(1)
    go_straight(4)#cd
    time.sleep(1)
def btoa_new():
    turn_right()
    time.sleep(1)
    go_straight(5.8)
    time.sleep(1)
    turn_right()
    time.sleep(1)
    go_straight(4)
    time.sleep(1)
    turn_right()
    time.sleep(1)
    turn_right()
    time.sleep(1)

def go_around_right():
    time.sleep(1)
    turn_right()
    time.sleep(1)
    go_straight_obs(1)
    time.sleep(1)
    turn_left()
    time.sleep(1)
    go_straight_obs(1.5)
    time.sleep(1)
    turn_left()
    time.sleep(1)
    go_straight_obs(1)
    time.sleep(1)
    turn_right()
    time.sleep(1)

def go_around_left():
    
    turn_left()
    time.sleep(1)
    go_straight_obs(1)
    time.sleep(1)
    turn_right()
    time.sleep(1)
    go_straight_obs(1.5)
    time.sleep(1)
    turn_right()
    time.sleep(1)
    go_straight_obs(1)
    time.sleep(1)
    turn_left()
    time.sleep(1)

def high_temp_htoe():
    # global turncount_init
    # turncount_init = 35*5
    go_straight(1)#ha
    time.sleep(1)
    turn_left()
    time.sleep(1)
    go_straight(3.6)#ab
    time.sleep(1)
    turn_left()
    time.sleep(1)
    go_straight(5)#bc
    time.sleep(1)
    turn_left()
    time.sleep(1)
    turn_left()#180
    time.sleep(1)
    go_straight(5)#cb
    time.sleep(1)
    turn_left()
    time.sleep(1)
    go_straight(4)#bd
    time.sleep(1)
    turn_left()
    time.sleep(1)
    go_straight(1)#de
    time.sleep(1)


def reset_voice_commands():
    r.set('forward','0')
    r.set('reverse','0')  
    r.set('left','0')  
    r.set('right','0')  
    r.set('clockwise','0')  
    r.set('anticlockwise','0')
    r.set('spin','0') 
    r.set('stop','0')   

def check_voice_command():
    #reset_voice_commands()
    if r.get('stop').decode('utf-8') == '1':
        r.set("robot","stop")
        r.set('stop','0')
    elif r.get('forward').decode('utf-8') == '1':
        go_straight(1)
        r.set('forward','0')
    elif r.get('reverse').decode('utf-8') == '1':
        go_reverse(1)
        r.set('reverse','0')  
    elif r.get('left').decode('utf-8') == '1':
        turn_left()
        time.sleep(1)
        r.set('left','0')  
    elif r.get('right').decode('utf-8') == '1':
        turn_right()
        time.sleep(1)
        r.set('right','0')
    elif r.get('clockwise').decode('utf-8') == '1':
        spin_clockwise360()
        r.set('clockwise','0')
    elif r.get('anticlockwise').decode('utf-8') == '1':
        spin_anticlockwise360()
        r.set('anticlockwise','0')
    elif r.get('spin').decode('utf-8') == '1':
        spin_clockwise360()
        time.sleep(0.3)    
        spin_anticlockwise360()
        r.set('spin','0')

#turn_right()
# spin_clockwise360()
# time.sleep(0.3)    
# spin_anticlockwise360()
# for _ in range(1):
    # atob_new()
    # time.sleep(1)
    # btoa_new()
    # time.sleep(2)

#turn_right_compass()
#turn('left',100)
#go_straight(24)
#go_around_left()
#turn_right()
# time.sleep(1)
#turn_left()
#atob_new()
#atob_new()
# time.sleep(2)
# atob_rev()
#high_temp_htoe()
#Multiiple path Navigation

xpaths = [['h', 'd'],['d','a'],['a','c'],['c','h']]
run = 1
init = 1

if enable_av == 1:
    reset_voice_commands()
    r.set("set","0")
    r.set('go','0')

    while robot.step(TIME_STEP) != -1:
        send_path = ""
        
        check_voice_command()
        
        if (r.get('set')).decode('utf-8') == '1':
            paths = (r.get('path')).decode('utf-8') #get path from TAB - eg "AB"
            if len(paths) > 1:
                print(paths)
                src = paths[0].lower()
                dst = paths[1].lower()
            else:
                continue
            if init == 1:
                heading = 'N'
                path, nav, heading = g.get_nav(src, dst, heading) #path is full path

                for s in path:#convert path in list form to a combined string
                    send_path += s
                r.set('pathList', send_path)#send full path

            else:
                path, nav, heading = g.get_nav(src, dst, heading)#path is full path
                for s in path:#convert path in list form to a combined string
                    send_path += s
                r.set('pathList', send_path) #send full path
            r.set("set","0")
            #print(nav)
        if (r.get('go')).decode('utf-8') == '1':
            init = 0
            for x in nav:
                print(x)
                if len(x) < 3:
                    r.set('nav', x) # send individual paths to tab - eg "Ha, ae"
                else:
                    exec(x) #execute navigation instructions
            r.set('go','0')
           
#Only simulator
else:
   

    while robot.step(TIME_STEP) != -1:

        if run == 1:
            for p in xpaths:
                print(p)
                src = p[0]
                dst = p[1]
                if init == 1:
                    heading = 'N'
                    path, nav, heading = g.get_nav(src, dst, heading)
                    init = 0
                else:
                    path, nav, heading = g.get_nav(src, dst, heading)
                for x in nav:
                    #print(x)
                    if len(x) > 3:
                        exec(x)
                time.sleep(5)
        run = 0
