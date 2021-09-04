#!/usr/bin/python3

import os, platform

from kivy.app import App
from numpy.core.defchararray import count
if platform.system() == "Linux" or platform.system() == "Darwin":
    os.environ["KIVY_VIDEO"] = "ffpyplayer"

from pysimbotlib.core import PySimbotApp, Robot, Simbot
from kivy.logger import Logger
from kivy.config import Config
import numpy as np
import time

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


odom = np.array([(20, 20, 0),(20, 20, 0),(20, 20, 0)]) # x, y, seta
odom_save = np.array([20, 20, 46.73570458892837]) # x, y, food_seta
odom_count = 0
count = 0

_near_dis = 7
_far_dis = 30
_near_wall_dis = 13
_center_wall_dis = 30


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
        Logger.info("Angle: {0}".format(self.smell()))

        # initial list of rules
        rules = list()
        turns = list()
        moves = list()

        # __________IR RULE__________
        rules.append(self.far_ir(0))
        turns.append(0)
        moves.append(10)

        rules.append(self.near_ir(0) * self.near_ir(1) * self.near_ir(-1))
        turns.append(0)
        moves.append(-10)

        rules.append(self.near_ir(0) * self.near_ir(1) * self.near_ir(-1) * 1.3)
        turns.append(self.smell_symbol() * 10)
        moves.append(0)

        rules.append(self.far_ir(0)* self.near_ir(1) * self.near_ir(2))
        turns.append(-5)
        moves.append(0)       

        rules.append(self.far_ir(0)* self.near_ir(-1) * self.near_ir(-2))
        turns.append(5)
        moves.append(0)

        # __________follow wall_________
        # rules.append(self.near_wall_ir(-2) * self.far_wall_ir(-1) * self.far_wall_ir(1) * self.far_ir(0))
        # turns.append(-0)
        # moves.append(5)        

        # rules.append(self.near_wall_ir(2) * self.far_wall_ir(-1) * self.far_wall_ir(1) * self.far_ir(0))
        # turns.append(0)
        # moves.append(5)        

        # __________smell rule__________
        rules.append(self.smell_left() * self.far_ir(0) * self.far_ir(1) * self.far_ir(-1) )
        turns.append(-10)
        moves.append(0)

        rules.append(self.smell_right() * self.far_ir(0) * self.far_ir(1) * self.far_ir(-1) )
        turns.append(10)
        moves.append(0)

        rules.append(self.smell_center() * self.far_ir(0))
        turns.append(0)
        moves.append(5)

        if self.check_struck == 0.1:
            rules.append(0.5)
            turns.append(10)
            moves.append(10)


        # rules.append(self.smell_left() * self.far_ir(0) * self.far_ir(1) * self.far_ir(2) * self.far_ir(-1) * self.far_ir(-2) * 2)
        # turns.append(-10)
        # moves.append(0)

        # rules.append(self.smell_right() * self.far_ir(0) * self.far_ir(1) * self.far_ir(2) * self.far_ir(-1) * self.far_ir(-2) * 2)
        # turns.append(10)
        # moves.append(0)

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
            return (irfront-_near_dis) / (_far_dis-_near_dis)
    
    def near_ir(self, x: int = 0):
        return 1 - self.far_ir(x) 

    def far_wall_ir(self, x: int = 0):
        global _near_wall_dis, _center_wall_dis
        irfront = self.ir_values[x]
        if irfront <= _near_wall_dis:
            return 0.0
        elif irfront >= _center_wall_dis:
            return 1.0
        else:
            return (irfront-_near_wall_dis) / (_center_wall_dis-_near_wall_dis)
    
    def near_wall_ir(self, x: int = 0):
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
        self.angle = 90.0
        target = self.smell()
        if target >= self.angle:
            return 1.0
        elif target <= 0:
            return 0.0
        else:
            return target / self.angle

    def smell_center(self):
        self.angle = 45.0
        target = abs(self.smell())
        if target >= self.angle:
            return 1.0
        elif target <= 0:
            return 0.0
        else:
            return target / self.angle

    def smell_left(self):
        self.angle = 90.0
        target = self.smell()
        if target <= - self.angle:
            return 1.0
        elif target >= 0:
            return 0.0
        else:
            return - target / self.angle

    def smell_symbol(self):
        self.angle = 90.0
        target = self.smell()
        if target > 0:
            return +1.0
        elif target < 0:
            return -1.0
        else:
            return 1.0

            
    # log linear odom data
    def move_s(self, x: int = 0):
        global odom
        if self.check_struck() == 1:
            self.move(x)
            self.loc_x = odom[-1][0] + x*np.cos(odom[-1][2] * np.pi /180.)
            self.loc_y = odom[-1][1] + x*np.sin(odom[-1][2] * np.pi /180.)
            self.new_odom_mat = [self.loc_x, self.loc_y, odom[-1][2]]
            odom = np.append(odom, [self.new_odom_mat], axis=0)

    # log angular odom data
    def turn_s(self, x:int = 0):
        global odom
        if self.check_struck() == 1:
            self.turn(x)    
            self.loc_z = odom[-1][2] + x 
            self.new_odom_mat = [odom[-1][0], odom[-1][1], self.loc_z]
            odom = np.append(odom, [self.new_odom_mat], axis=0)

    # know the robot actually struck or not
    def check_struck(self):
        global _struct, _count_stuck, odom
        self.diff_odom = np.round(np.average(odom[-3][0:2]) - np.average(odom[-1][0:2]),1)
        # print("diff_odom: {0}".format(self.diff_odom))
        if self.diff_odom <= 4 and self.stuck:
            return 0
        else:
            return 1

if __name__ == '__main__':
    # app = PySimbotApp(FuzzyRobot, ROBOT_NUM, mapPath=MAP_FILE, interval=TIME_INTERVAL, maxtick=MAX_TICK)
    app = PySimbotApp(robot_cls=FuzzyRobot, num_robots=1, interval=TIME_INTERVAL, enable_wasd_control=True,save_wasd_history=True)
    app.run()