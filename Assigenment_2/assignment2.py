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

        # initial list of rules
        rules = list()
        turns = list()
        moves = list()

        rules.append(self.front_far() * self.left_far() * self.right_far())
        turns.append(0)
        moves.append(10)

        rules.append(self.front_near() * self.left_near() * self.right_near())
        turns.append(0)
        moves.append(-10)

        rules.append(self.front_near() * self.left_near() * self.right_far())
        turns.append(60)
        moves.append(0)        
        
        rules.append(self.front_near() * self.left_far() * self.right_near())
        turns.append(-60)
        moves.append(0)

        rules.append(self.smell_center() * self.front_far())
        turns.append(0)
        moves.append(10)

        rules.append(self.smell_left() * self.front_far() * self.left_far() * self.right_far())
        turns.append(-45)
        moves.append(5)

        rules.append(self.smell_right() * self.front_far() * self.left_far() * self.right_far())
        turns.append(60)
        moves.append(5)

        ans_turn = 0.0
        ans_move = 0.0
        for r, t, m in zip(rules, turns, moves):
            ans_turn += t * r
            ans_move += m * r

        self.turn(ans_turn)
        self.move(ans_move)
        
    def front_far(self):
        irfront = self.ir_values[0]
        if irfront <= 10:
            return 0.0
        elif irfront >= 40:
            return 1.0
        else:
            return (irfront-10.0) / 30.0
    
    def front_near(self):
        return 1.0 - self.front_far()

    def left_far(self):
        irleft = self.ir_values[6]
        if irleft <= 10:
            return 0.0
        elif irleft >= 30: 
            return 1.0
        else:
            return (irleft-10.0) / 20.0

    def left_near(self):
        return 1 - self.left_far()

    def right_far(self):
        irright = self.ir_values[2]
        if irright <= 10:
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