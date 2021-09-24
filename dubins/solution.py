#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Yage Hao}
# {19980707-9547}
# {yage@kth.se}

from dubins import *
import math 

class Node(object):
    def __init__(self, car, x, y, theta, cost_past=0, parent=None):
        self.car = car 
        self.x = x 
        self.y = y
        self.theta = theta 
        self.cost_past = cost_past 
        self.parent = parent 

        # init in set_cost() function
        self.cost_future = 0
        self.total_cost = 0 

        # init in next_node function 
        self.time = 0 
        self.phi = 0 

    def set_cost(self):
        """compute cost heuristic to goal by using Euclidean function."""
        self.cost_future = math.hypot((self.x - self.car.xt), (self.y - self.car.yt)**2)
        self.total_cost = 0.5 * self.cost_past + self.cost_future

    def isgoal(self):
        """if distance within 1.0, we reach the goal."""
        distance = math.sqrt(((self.x - self.car.xt)**2 + (self.y - self.car.yt)**2))
        if distance <= 1.0:
            return True 
        else:
            return False 

def not_collide(car, x, y):
    """check if the car collide with boundary or obstacles."""
    if x > car.xub:
        return False 
    if x < car.xlb:
        return False 
    if y > car.yub - 0.2:
        return False 
    if y < car.ylb + 0.1:
        return False 
    for ob in car.obs:
        if math.hypot(x-ob[0], y-ob[1]) < ob[2] + 0.15:
            return False 
    return True 

def index(node):
    x_index = int(node.x/0.3) # linear resolution 0.1
    y_index = int(node.y/ 0.3)
    theta_index = int(node.theta % (2*math.pi)) / (math.pi/3) # angular resolution pi/6
    return (x_index, y_index, theta_index) 

def  next_node(car, current_node, open_set, closed_set, timesteps = 50):
    steer_direction = [-math.pi/4, 0, math.pi/4] 
    for direction in steer_direction:
        x_i = current_node.x 
        y_i = current_node.y 
        theta_i = current_node.theta 

        for i in range(timesteps): # iterative steps
            iterater = i # max 50 
            x_i, y_i, theta_i = step(car, x_i, y_i, theta_i, direction)
            if not not_collide(car, x_i, y_i): # if collision
                iterater = 999 
                break 
        
        # find next node to check 
        next_node = Node(car, x_i, y_i, theta_i)
        next_index = index(next_node)
        if next_index not in closed_set:
            if iterater == 999: # collide and break 
                closed_set[next_index] = next_node 
            elif iterater == timesteps-1:
                next_node.cost_past = current_node.cost_past + 0.01*timesteps
                next_node.set_cost()
                next_node.parent = current_node 
                next_node.time = current_node.time + timesteps 
                next_node.phi = direction 
                if next_index in open_set:
                    if next_node.total_cost < open_set[next_index].total_cost:
                        open_set[next_index] = next_node 
                else:
                    open_set[next_index] = next_node 
    return True 


def solution(car):

    ''' <<< write your code below >>> '''
    controls=[]
    times=[]
    theta0 = 0
    initial_node = Node(car, car.x0, car.y0, theta0)
    initial_node.set_cost()
    open_set = {index(initial_node) : initial_node}
    closed_set = {}

    while len(open_set) != 0:
        current_index = min(open_set, key=lambda l: open_set[l].total_cost)
        current_node = open_set[current_index]
        del open_set[current_index]
        closed_set[current_index] = current_node

        if current_node.isgoal():
            times.insert(0, current_node.time * 0.01)
            controls.insert(0, current_node.phi)
            
            #trace back until reaching the initial node 
            while True:
                if current_node.parent == None:
                    controls.pop(0)
                    break 
                else:
                    current_node = current_node.parent
                    times.insert(0, current_node.time * 0.01)
                    controls.insert(0, current_node.phi)
            break 
        else:
            next_node(car, current_node, open_set, closed_set) 
    ''' <<< write your code above >>> '''

    return controls, times


