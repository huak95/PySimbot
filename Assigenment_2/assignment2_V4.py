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

# Force the program to show user's log only for "info" level or more. The info log will be disabled.
Config.set('kivy', 'log_level', 'info')
# START POINT
START_POINT = (20, 560)

# update robot every 0.5 seconds (2 frames per sec)
REFRESH_INTERVAL = 1/10

class MyRobot(Robot):

    def __init__(self):
        super(MyRobot, self).__init__()
        self.pos = START_POINT

    def update(self):
        ''' Update method which will be called each frame
        '''        
        self.ir_values = self.distance()
        self.target = self.smell()
        Logger.info("Distance: {0}".format(self.distance()))

        # initial list of rules
        rules = list()
        turns = list()
        moves = list()

        # __move forward when front is far__

        rules.append(self.far_ir(0) * self.far_ir(-2) * self.far_ir(2))
        moves.append(15)
        turns.append(0)

        # rules.append(self.far_ir(0) * self.near_ir(-2) * self.near_ir(2))
        # moves.append(10)
        # turns.append(0)

        rules.append(self.far_ir(0)* np.min([self.near_ir(-2),self.near_ir(2)]))
        moves.append(10)
        turns.append(0)

        rules.append(self.near_ir(0))
        moves.append(3)
        turns.append(0)
        

        # __move back for safety__
        rules.append(self.near_ir(0))
        moves.append(-5)
        turns.append(0)

        rules.append(self.near_ir(0) * self.near_ir(-1) * self.near_ir(1))
        moves.append(-5)
        turns.append(0)

        # back sensor
        rules.append(self.near_ir(3) * self.near_ir(4) * self.near_ir(5) * self.far_ir(0))
        moves.append(3)
        turns.append(0)

        # right sensor 
        rules.append(self.near_ir(-2) * self.far_ir(2))
        moves.append(3)
        turns.append(15)
        
        rules.append(self.near_ir(-1) * self.far_ir(1))
        moves.append(3)
        turns.append(10)

        # left sensor
        rules.append(self.far_ir(-2) * self.near_ir(2))
        moves.append(3)
        turns.append(-15)

        rules.append(self.far_ir(-1) * self.near_ir(1))
        moves.append(3)
        turns.append(-10)

        # __________FOOD RULE__________

        # smell center move
        rules.append(self.smell_center() * self.far_ir(0) )
        moves.append(10)
        turns.append(0)

        rules.append(self.smell_left() * self.far_ir(0) * self.far_ir(-2) * self.far_ir(2))
        moves.append(0)
        turns.append(-12)

        rules.append(self.smell_right() * self.far_ir(0) * self.far_ir(-2) * self.far_ir(2))
        moves.append(0)
        turns.append(12)

        ans_turn = 0.0
        ans_move = 0.0
        for r, t, m in zip(rules, turns, moves):
            ans_turn += t * r
            ans_move += m * r

        self.turn(int(ans_turn))
        self.move(int(ans_move))
    
        count += 1

    def near_ir(self, x):
        self.ir = self.ir_values[x]
        if x%2 == 0:
            self.x1 = 8.0
            self.x2 = 40.0
        else:
            self.x1 = 12.0
            self.x2 = 30.0

        if self.ir <= self.x1:
            return 1.0
        elif self.ir >= self.x2:
            return 0.0
        else:
            return (self.ir-self.x1)/(self.x2-self.x1)

    def mid_ir(self, x):
        self.ir = self.ir_values[x]       
        self.x1 = 10.0
        self.x2 = 30.0
        self.x3 = 60.0
        if self.ir <= self.x1:
            return 0.0
        elif self.ir > self.x1 and self.ir < self.x2:
            return (self.ir-self.x1)/(self.x2-self.x1)
        elif self.ir >= self.x2 and self.ir < self.x3:
            return 1.0 - (self.ir-self.x2)/(self.x3-self.x2)
        else:
            return 0.0

    def far_ir(self, x):
        return 1.0 - self.near_ir(x)
    
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
            return 0.0
        elif target <= 0:
            return 1.0
        else:
            return 1.0 - (target / 45.0)

    def smell_left(self):
        target = self.smell()
        if target <= -90:
            return 1.0
        elif target >= 0:
            return 0.0
        else:
            return -target / 90.0

if __name__ == '__main__':
    app = PySimbotApp(map="default", robot_cls=MyRobot, num_robots=1, interval=REFRESH_INTERVAL, enable_wasd_control=True,save_wasd_history=True)
    app.run()