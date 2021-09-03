#!/usr/bin/python3

from periphery import GPIO 
import time                  
import redis
import os
import subprocess as sp

from time import sleep 
try:
	r = redis.Redis(host = 'localhost', port = 6379)
	r.set("robot","stop")
	r.set("obs", "false")
	print('database connected')
except Exception as e:
        print("Exception@DBconnect",e.message)

try:
	pin37 = GPIO(26,"out")
	pin1 = GPIO( 5, "out")
	pin2 = GPIO( 6, "out")
	pin5 = GPIO( 24, "in")
	pin3 = GPIO( 22, "out")
	pin4 = GPIO( 27, "out")
	print('gpio pin enabled')
except Exception as e:
        print("Exception@gpio",e.message)

S_FLAG = 0
while True:

	robot = r.get("robot")
	ip = "localhost"
	status, result = sp.getstatusoutput("ping -c1 -w1" +ip)
	if status == 0:
		r.set("robot","stop")
	emergency = pin5.read()
	r.set("emergency",emergency)
	if emergency == False:
		r.set("robot","stop")

	speed = r.get("speed")
	if speed == b"1":
		pin37.write(True)
	else:
		pin37.write(False)
	if robot == b"stop" and S_FLAG == 1:
		print("STOP")
		pin1.write(False)
		pin2.write(False)
		pin3.write(False)
		pin4.write(False)
		S_FLAG = 0
	elif robot == b"reverse" and S_FLAG == 0:
		print("REV")
		pin1.write(True)
		pin3.write(True)
		pin4.write(True)
		S_FLAG = 1
	elif robot == b"forward" and S_FLAG == 0 and r.get("obs") !="true":
		print("FWD")
		pin1.write(True)
		pin2.write(True)
		pin3.write(True)
		S_FLAG = 1
	elif robot == b"right" and S_FLAG == 0:
		print("RIGHT")
		pin1.write(True)
		pin3.write(True)
		S_FLAG = 1
	elif robot == b"left" and S_FLAG == 0:
		print("LEFT")
		pin1.write(True)
		pin2.write(True)
		pin3.write(True)
		pin4.write(True)
		S_FLAG = 1 	

