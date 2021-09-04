#!/usr/bin/python3

import os, sys
import random
import numpy as np

from pysimbotlib.Window import PySimbotApp
from pysimbotlib.Robot import Robot
from kivy.core.window import Window
from kivy.logger import Logger


# Number of robot that will be run
ROBOT_NUM = 1

# Delay between update (default: 1/60 (or 60 frame per sec))
TIME_INTERVAL = 1.0/10 #10frame per second 

# Max tick
MAX_TICK = 5000

# START POINT
START_POINT = (20, 560)

# Map file
MAP_FILE = 'maps/default_map.kv'


odom = np.array([(20, 20, 0),(20, 20, 0)]) # x, y, seta
odom_save = np.array([20, 20, 46.73570458892837]) # x, y, food_seta
odom_count = 0
count = 0

_near_dis = 10
_far_dis = 50


_count_lim = 0 
_count_set = 0
_ro = 1
_count_stuck = 0
_count_hold = 8
_sum_odom = 0
_sum_odom_pre = 0
_struct = 0
SENSOR = [0]*8
smell_dis =0 

class FuzzyRobot(Robot):

    def __init__(self):
        super(FuzzyRobot, self).__init__()
        self.pos = START_POINT

    def update(self):
        global odom, count, odom_save, odom_count
        global _count_lim, _count_set, _ro, _count_stuck, _count_hold, _sum_odom, _sum_odom_pre, _struct, SENSOR, smell_dis

        ''' Update method which will be called each frame
        '''        
        self.ir_values = self.distance()
        self.target = self.smell()
        Logger.info("Distance: {0}".format(self.distance()))
        Logger.info("Odom_Now: {0}".format(odom[-1][:]))
        Logger.info("Stuck: {0}".format(self.check_struck()))

        # initial list of rules
        rules = list()
        turns = list()
        moves = list()

        # __________IR RULE__________
        rules.append(self.far_ir(0))
        turns.append(0)
        moves.append(10)

        rules.append(self.near_ir(0))
        turns.append(15)
        moves.append(-3)

        rules.append(self.near_ir(1) * self.near_ir(2))
        turns.append(-15)
        moves.append(0)       

        rules.append(self.near_ir(-1) * self.near_ir(-2))
        turns.append(15)
        moves.append(0)

        # rules.append(self.near_ir(1) * self.near_ir(2) * self.far_ir(-1) * self.far_ir(-2))
        # turns.append(15)
        # moves.append(0)       

        # rules.append(self.near_ir(-1) * self.near_ir(-2) * self.far_ir(1) * self.far_ir(1))
        # turns.append(-15)
        # moves.append(0)

        # rules.append(self.front_far() * self.right_near() * self.left_near())
        # turns.append(0)
        # moves.append(2)        


        # __________smell rule__________
        rules.append(self.smell_left() * self.far_ir(0) * self.far_ir(1) * self.far_ir(2) * self.far_ir(-1) * self.far_ir(-2) )
        turns.append(-15)
        moves.append(0)

        rules.append(self.smell_right() * self.far_ir(0) * self.far_ir(1) * self.far_ir(2) * self.far_ir(-1) * self.far_ir(-2) )
        turns.append(15)
        moves.append(0)

        ans_turn = 0.0
        ans_move = 0.0
        for r, t, m in zip(rules, turns, moves):
            ans_turn += t * r
            ans_move += m * r

        self.turn_s(ans_turn)
        self.move_s(ans_move)
        
    def far_ir(self, x: int = 0):
        irfront = self.ir_values[x]
        if irfront <= _near_dis:
            return 0.0
        elif irfront >= _far_dis:
            return 1.0
        else:
            return (irfront-_near_dis) / 30.0
    
    def near_ir(self, x: int = 0):
        return 1 - self.far_ir(x) 

    def right_far(self):
        irright = self.ir_values[2]
        if irright <= 6:
            return 0.0
        elif irright >= 30:
            return 1.0
        else:
            return (irright-10.0) / 20.0

    def right_near(self):
        return 1 - self.right_far()
    
    def smell_right(self):
        target = self.smell()
        if target >= 90:
            return 1.0
        elif target <= 0:
            return 0.0
        else:
            return target / 90.0



    def smell_center(self):
        target = abs(self.smell())
        if target >= 45:
            return 1.0
        elif target <= 0:
            return 0.0
        else:
            return target / 45.0

    def smell_left(self):
        target = self.smell()
        if target <= -90:
            return 1.0
        elif target >= 0:
            return 0.0
        else:
            return -target / 90.0
            
    # log linear odom data
    def move_s(self, x: int = 0):
        global odom
        if self.check_struck() == False:
            self.move(x)
            self.loc_x = odom[-1][0] + x*np.cos(odom[-1][2] * np.pi /180.)
            self.loc_y = odom[-1][1] + x*np.sin(odom[-1][2] * np.pi /180.)
            self.new_odom_mat = [self.loc_x, self.loc_y, odom[-1][2]]
            odom = np.append(odom, [self.new_odom_mat], axis=0)

    # log angular odom data
    def turn_s(self, x:int = 0):
        global odom
        if self.check_struck() == False:
            self.turn(x)    
            self.loc_z = odom[-1][2] + x 
            self.new_odom_mat = [odom[-1][0], odom[-1][1], self.loc_z]
            odom = np.append(odom, [self.new_odom_mat], axis=0)

    # know the robot actually struck or not
    def check_struck(self):
        global _struct, _count_stuck, odom
        self.diff_odom = np.round(np.average(odom[-2][0:2]) - np.average(odom[-1][0:2]),1)
        # print("diff_odom: {0}".format(self.diff_odom))
        if self.diff_odom <= 1 :
            return False
        else:
            return False

if __name__ == '__main__':
    app = PySimbotApp(FuzzyRobot, ROBOT_NUM, mapPath=MAP_FILE, interval=TIME_INTERVAL, maxtick=MAX_TICK)
    app.run()