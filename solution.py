#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {xuehao zhang}
# {xuehao@kth.se}
from dubins import *
import math as m
from numpy import inf

car=Car()

def Clash(x,y,car):
    for obs in car.obs:
        d = m.sqrt((x - obs[0])**2 + (y - obs[1])**2)
        if d <= obs[2] + 0.1:
            return True
    return False

def Block(x,y,car):
    if (car.xlb <= x <= car.xub and car.ylb <= y <= car.yub ):
        return False
    return True
   
def new_path(x,y,phi,theta,car,controls,times,limit):
    cost = 0
    for i in range(100 if phi == 0 else 157):
        dt=0.01
        x, y, theta = step(car, x, y, theta, phi)
        while theta >= m.pi:
            theta -= 2*m.pi
        while theta <= -2*m.pi:
            theta += m.pi  
        controls.append(phi)
        times.append(times[-1] + dt)
        if Clash(x,y,car) or Block(x,y,car):
            return False, 0, 0, 0, controls, times, inf
        if m.sqrt((x - car.xt)**2 + (y - car.yt)**2) <= limit:
            return True, x, y, theta, controls, times, 0
    cost = m.sqrt((x - car.xt)**2 + (y - car.yt)**2)
    return True, x, y, theta, controls, times, cost
   
def First_discover(car, path, checked):
    limit = 0.2
    queue = [[car.x0,car.y0,0,[],[0],m.sqrt((car.x0 - car.xt)**2 + (car.y0 - car.yt)**2)]]
    queue1 = []
    while len(queue) > 0:
        x,y,theta,controls,times,_ = queue.pop(0)
        if m.sqrt((x - car.xt)**2 + (y - car.yt)**2) <= limit:
            return controls, times
        checked.append([round(x,1), round(y,1)])
        for phi in [-m.pi/4, 0, m.pi/4]:
            useable, x1, y1, theta1, controls1, times1, cost = new_path(x,y,phi,theta,car,new_array(controls),new_array(times),limit)
            if useable and not [round(x1,1), round(y1,1), round(theta1,1)] in queue1:
                path.append(phi)
                queue1.append([round(x1,1), round(y1,1), round(theta1,1)])
                queue.append([x1, y1, theta1, controls1, times1, cost])
            queue.sort(key=lambda x: x[5])
    return [],[0]

def new_array(arr):
    new_arr = []
    for x in arr:
        new_arr.append(x)
    return new_arr

def solution(car):
    controls=[0]
    times=[0,0.01]
    controls, times = First_discover(car, [], [])
    return controls, times