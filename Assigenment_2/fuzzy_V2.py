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

odom = np.array([(20, 20, 0),(20, 20, 0)]) # x, y, seta
odom_save = np.array([20, 20, 46.73570458892837]) # x, y, food_seta
odom_count = 0
count = 0

_near_dis = 10
_far_dis = 50

# Force the program to show user's log only for "info" level or more. The info log will be disabled.
Config.set('kivy', 'log_level', 'info')

# update robot every 0.5 seconds (2 frames per sec)
REFRESH_INTERVAL = 1/10

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


class MyRobot(Robot):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.safety_dist = 30
        self.closed_dist = 10
        self.hitted_dist = 0
        self.diff_odom = 0

    def update(self):
        global odom, count, odom_save, odom_count
        global _count_lim, _count_set, _ro, _count_stuck, _count_hold, _sum_odom, _sum_odom_pre, _struct, SENSOR, smell_dis
        global food_loc, food_dis
        # _sum_odom_pre = round(sum(odom[0:1],0))

        SENSOR = self.distance()
        self.ir_values = self.distance()
        self.target = self.smell()
        # Logger.info("Smell Angle: {0}".format(self.smell()))
        # Logger.info("Odom_Now: {0}".format(odom[-1][:]))
        # Logger.info("Odom_Pre: {0}".format(odom[-2][:]))
        Logger.info("Distance: {0}".format(self.distance()))
        # Logger.info("Stuck: {0}".format(self.check_struck()))
        
        if count == 0:
            food_loc = np.array(self.food_finding_mode_x())
            
        food_dis = self.food_len()
        # print("food_dis: {0}".format(food_dis))

        self.ir_dis_var = 10

        self.OAC()
        # if (self.ir_values[-2]<=self.ir_dis_var or self.ir_values[-1]<=self.ir_dis_var or self.ir_values[0]<=self.ir_dis_var or self.ir_values[1]<=self.ir_dis_var or self.ir_values[2]<=self.ir_dis_var) :
        #     # print("Object Avoidance Mode!")
        #     self.OAC()
        # else:
        #     # print("Food Finding Mode!")
        #     self.FLC()              

        count += 1

    def FLC(self):
        # initial list of rules
        rules = list()
        turns = list()
        moves = list()
        # __________FOOD RULE__________
        rules.append(self.food_dis_zero() * 1.0)
        moves.append(0)
        turns.append(0)

        rules.append(self.food_dis_near() * self.food_angle_center())
        moves.append(10)
        turns.append(0)

        rules.append(self.food_dis_far() * self.food_angle_center())
        moves.append(10)
        turns.append(0)

        rules.append(1.0 * self.food_angle_left())
        moves.append(0)
        turns.append(-20)

        rules.append(1.0 * self.food_angle_right())
        moves.append(0)
        turns.append(20)

        ans_turn = 0.0
        ans_move = 0.0

        for r, t, m in zip(rules, turns, moves):
            ans_turn += t * r
            ans_move += m * r

        # print("ans_turn: {0} \t ans_move: {1}".format(ans_turn, ans_move))
        self.turn_s(int(ans_turn))
        self.move_s(int(ans_move))

    def OAC(self):
        # initial list of rules
        rules = list()
        turns = list()
        moves = list()
        # __________IR RULE__________
        self.ARR = np.array([self.close_ir(-2), self.close_ir(-1), self.close_ir(0)])        
        rules.append(np.amax(self.ARR))
        moves.append(5)
        turns.append(10)

        self.ARR = np.array([self.close_ir(0), self.close_ir(1) , self.close_ir(2)])
        rules.append(np.amax(self.ARR))
        moves.append(-5)
        turns.append(-10)
        
        self.ARR = np.array([self.close_ir(-2), self.near_ir(-1), self.near_ir(0)])
        rules.append(np.amax(self.ARR))
        moves.append(5)
        turns.append(20)

        self.ARR = np.array([self.close_ir(2), self.near_ir(1), self.near_ir(0)])
        rules.append(np.amax(self.ARR))
        moves.append(-5)
        turns.append(-20)

        self.ARR = np.array([self.far_ir(-2), self.far_ir(-1), self.far_ir(0), self.far_ir(1), self.far_ir(2)])
        rules.append(np.amax(self.ARR))
        moves.append(-20)
        turns.append(0)

        ans_turn = 0.0
        ans_move = 0.0

        for r, t, m in zip(rules, turns, moves):
            ans_turn += t * r
            ans_move += m * r

    def food_dis_zero(self):
        self.x1 = 50.0
        self.x2 = 100.0
        if food_dis <= self.x1:
            return 1.0
        elif food_dis >= self.x2:
            return 0.0
        else:
            return (food_dis-self.x1)/(self.x2-self.x1)

    def food_dis_near(self):
        self.x1 = 40.0
        self.x2 = 100.0
        self.x3 = 400.0
        if food_dis <= self.x1:
            return 0.0
        elif food_dis > self.x1 and food_dis < self.x2:
            return (food_dis-self.x1)/(self.x2-self.x1)
        elif food_dis == self.x2:
            return 1.0
        elif food_dis >= self.x2 and food_dis < self.x3:
            return 1.0 - (food_dis-self.x2)/(self.x3-self.x2)
        else:
            return 0.0

    def food_dis_far(self):
        self.x1 = 100.0
        self.x2 = 400.0
        if food_dis <= self.x1:
            return 0.0
        elif food_dis >= self.x2:
            return 1.0
        else:
            return 1.0 - (food_dis-self.x1)/(self.x2-self.x1)

    def food_angle_left(self):
        target = self.smell()
        if target <= -90:
            return 1.0
        elif target >= 0:
            return 0.0
        else:
            return -target / 90.0

    def food_angle_right(self):
        target = self.smell()
        if target >= 90:
            return 1.0
        elif target <= 0:
            return 0.0
        else:
            return target / 90.0

    def food_angle_center(self):
        target = abs(self.smell())
        if target >= 45:
            return 0.0
        elif target <= 0:
            return 1.0
        else:
            return 1.0 - (target / 45.0)

    def close_ir(self, x):
        self.ir = self.ir_values[x]
        self.x1 = 5.0
        self.x2 = 15.0
        if self.ir <= self.x1:
            return 1.0
        elif self.ir >= self.x2:
            return 0.0
        else:
            return (self.ir-self.x1)/(self.x2-self.x1)
 
    def near_ir(self, x):
        self.ir = self.ir_values[x]       
        self.x1 = 10.0
        self.x2 = 30.0
        self.x3 = 60.0
        if self.ir <= self.x1:
            return 0.0
        elif self.ir > self.x1 and self.ir < self.x2:
            return (self.ir-self.x1)/(self.x2-self.x1)
        elif self.ir == self.x2:
            return 1.0
        elif self.ir >= self.x2 and self.ir < self.x3:
            return 1.0 - (self.ir-self.x2)/(self.x3-self.x2)
        else:
            return 0.0

    def far_ir(self, x):
        self.ir = self.ir_values[x]
        self.x1 = 25.0
        self.x2 = 60.0
        if self.ir <= self.x1:
            return 0.0
        elif self.ir >= self.x2:
            return 1.0
        else:
            return 1.0 - (self.ir-self.x1)/(self.x2-self.x1)

    def food_finding_mode_x(self):
        self.theta1 = (90.0 - self.smell())*np.pi/180.
        self.move_s(10)
        self.dx = 10.0
        self.theta2 = (90.0 - self.smell())*np.pi/180.
        self.tan_var = (np.tan(self.theta1)/np.tan(self.theta2))-1.0
        self.food_x = np.round(self.dx/np.round(self.tan_var,4),4)
        self.food_y = np.round(self.food_x / np.tan(self.theta2),4)
        self.move_s(-10)
        return self.food_x, self.food_y

    def food_len(self):
        self.food_loc = food_loc
        # print("food_loc: {0}".format(food_loc))
        self.dx = np.abs(self.food_loc[0] - odom[-1][0])
        self.dy = np.abs(self.food_loc[1] - odom[-1][1])
        self.food_dis = np.sqrt(np.power(self.dx,2) + np.power(self.dy,2))
        # print("food_dis: {0}".format(self.food_dis))
        return self.food_dis
    
    def obstracle_avoid_mode(self):
        # print("obstracle_avoid_mode ACTIVATE!!")
        global SENSOR
        if(SENSOR[0] >= self.safety_dist and SENSOR[1] >= self.closed_dist and SENSOR[-1] >= self.closed_dist and not self.check_struck()):
            self.move_s(5)

        if(SENSOR[0] >= self.safety_dist and (SENSOR[1] < self.closed_dist) or (SENSOR[-1] < self.closed_dist and not self.check_struck())):
            if (SENSOR[1]<SENSOR[-1]):
                self.turn_s(-5)
            elif(SENSOR[1]>SENSOR[-1]):
                self.turn_s(5)

        if(SENSOR[0] < self.safety_dist and not self.check_struck()):
            self.turn_s(4)

    
    def odom_go_to_len(self, x):
        self.odom_arr = np.arange(0,x,5)
        self.odom_arr = np.append(self.odom_arr,[x])
        for i in range(1,len(self.odom_arr)):
            self.odom_arr_cut = self.odom_arr[i] - self.odom_arr[i-1]
            # print("self.odom_arr_cut: {0}".format(self.odom_arr_cut))
            self.move_s(self.odom_arr_cut)
            time.sleep(0.0001)

        self.move_s(self.odom_y_len)

    def load_save(self):
        global odom_save, odom, odom_count
        self.turn_s(180)
        self.odom_diff = (odom[-1][0] - odom_save[0])
        # print("odom_diff")
        # print(self.odom_diff)
        self.move_s(self.odom_diff)
        self.turn_s(180)
        odom_count = 0    
        
        # while True:
        #     if odom[-1] <= odom_save:
        #         break
        #     else:
        #        self.move_s(5)

    # log linear odom data
    def move_s(self, x):
        global odom
        if self.check_struck() == False:
            self.move(x)
            self.loc_x = odom[-1][0] + x*np.cos(odom[-1][2] * np.pi /180.)
            self.loc_y = odom[-1][1] + x*np.sin(odom[-1][2] * np.pi /180.)
            self.new_odom_mat = [self.loc_x, self.loc_y, odom[-1][2]]
            odom = np.append(odom, [self.new_odom_mat], axis=0)

    # log angular odom data
    def turn_s(self, x):
        global odom
        if self.check_struck() == False:
            self.turn(x)    
            self.loc_z = odom[-1][2] + x 
            self.new_odom_mat = [odom[-1][0], odom[-1][1], self.loc_z]
            odom = np.append(odom, [self.new_odom_mat], axis=0)
            # print("x: {0}".format(x))

            

    # know the robot actually struck or not
    def check_struck(self):
        global _struct, _count_stuck, odom
        self.diff_odom = np.round(np.average(odom[-2][0:2]) - np.average(odom[-1][0:2]),1)
        # print("diff_odom: {0}".format(self.diff_odom))
        if self.diff_odom <= 1 and self.stuck:
            return True
        else:
            return False
            


if __name__ == '__main__':
    app = PySimbotApp(map="default", robot_cls=MyRobot, num_robots=1, interval=REFRESH_INTERVAL, enable_wasd_control=True,save_wasd_history=True)
    app.run()