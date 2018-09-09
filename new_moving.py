#!/usr/bin/env python3

import math
from threading import Thread
from time import sleep, time

from ev3dev.ev3 import *


class leds:
    def __init__(self):
        self.leds = Leds()

    def led_off(self):
        self.leds.led_off()

    def red(self, side):
        if (side == "left"):
            self.leds.set_color(self.leds.LEFT, self.leds.RED)
        if (side == "right"):
            self.leds.set_color(self.leds.RIGHT, self.leds.RED)
        if (side == "both"):
            self.leds.set_color(self.leds.LEFT, self.leds.RED)
            self.leds.set_color(self.leds.RIGHT, self.leds.RED)

    def green(self, side):
        if (side == "left"):
            self.leds.set_color(self.leds.LEFT, self.leds.GREEN)
        if (side == "right"):
            self.leds.set_color(self.leds.RIGHT, self.leds.GREEN)
        if (side == "both"):
            self.leds.set_color(self.leds.LEFT, self.leds.GREEN)
            self.leds.set_color(self.leds.RIGHT, self.leds.GREEN)


class lcd:
    """lcd class allows to control lcd display ev3 brick """

    def __init__(self):
        self.lcd = Screen()

    def lcd_clear(self):
        self.lcd.clear()
        self.lcd.update()


class button:
    """button class allws to control buttons on ev3 brick"""

    def __init__(self):
        self.btn = Button()
        print("class inizialized")

    def waitButtons(self):
        while True:
            if self.btn.any():
                return 0
            else:
                sleep(0.01)


class Motor:
    def __init__(self):
        self.robot = {
            'vBase': 51,
            'wheelD': 8.16,
            'track': 19.5,
            'leftMotor_reverse': -1,
            'rightMotor_reverse': 1,
            'cpr': 360
        }
        try:
            left = open('left_sensor.txt')
            arr1 = []
            for line in left:
                arr1.append(int(line))
            right = open('right_sensor.txt')
            arr2 = []
            for line in right:
                arr2.append(int(line))
        except:
            Sound.beep().wait()
            sleep(0.5)
            Sound.beep().wait()
            sleep(0.5)
            Sound.beep().wait()
            sleep(0.5)
            arr1 = [40, 50, 60]
            arr2 = [40, 50, 60]
        self.left = {
            'black': arr1[0] + 5,
            'white': arr1[1] - 5,
            'gray': arr1[2] + 3
        }
        self.right = {
            'black': arr2[0] + 5,
            'white': arr2[1] - 5,
            'gray': arr2[2] + 3
        }
        self.mLeft = MediumMotor('outB')
        self.mRight = MediumMotor('outC')
        self.mLeft.reset()
        self.mRight.reset()
        self.reading_left_encoder = False
        self.reading_right_encoder = False
        self.encL = 0
        self.encR = 0

    def reset(self):
        self.mLeft.reset()
        self.mRight.reset()
        self.reading_Left_encoder = False
        self.reading_right_encoder = False

    def sign(self, num):
        # returns 1 if number positive, -1 if negative
        return(1 if num > 0 else -1)

    def motors(self, vLeft, vRight):
        if vLeft > 100:
            vLeft = 100
        if vLeft < -100:
            vLeft = -100
        if vRight > 100:
            vRight = 100
        if vRight < -100:
            vRight = -100
        self.mLeft.run_direct(duty_cycle_sp=vLeft *
                              self.robot['leftMotor_reverse'])
        self.mRight.run_direct(duty_cycle_sp=vRight *
                               self.robot['rightMotor_reverse'])

    def motors_forever(self, vLeft, vRight):
        sp = 900
        if vLeft > sp:
            vLeft = sp
        if vLeft < -sp:
            vLeft = -sp
        if vRight > sp:
            vRight = sp
        if vRight < -sp:
            vRight = -sp
        self.mLeft.run_forever(
            speed_sp=vLeft * self.robot['leftMotor_reverse'])
        self.mRight.run_forever(
            speed_sp=vRight * self.robot['rightMotor_reverse'])

    def stop(self):
        self.mLeft.stop(stop_action="hold")
        self.mRight.stop(stop_action="hold")
        # sleep(0.01)
        self.mLeft.run_direct(duty_cycle_sp=0)
        self.mRight.run_direct(duty_cycle_sp=0)

    def sm2cpr(self, sm):
        # counting motor degrees
        return sm/self.robot['wheelD']/math.pi*self.robot['cpr']

    def leftEncoder(self):
        while self.reading_left_encoder:
            self.encL = abs(int(self.mLeft.position))
            sleep(0.001)
    def rightEncoder(self):
        while self.reading_right_encoder:
            self.encR = abs(int(self.mRight.position))
            sleep(0.001)
    def MoveEncNew(self,sm, stop = True,speed=90):
        self.mLeft.position = 0
        self.mRight.position = 0
        encL = abs(self.mLeft.position)
        encR = abs(self.mRight.position)
        startL = encL
        startR = encR
        nB = int(self.sm2cpr(abs(sm)) + (startL + startR)/2)
        self.motors(speed, speed)
        while (encL + encR >> 1 <= nB):
            sleep(0.001)
        if stop:
            self.motors(0, 0)
            self.stop()        

    def moveEnc(self, sm, stop=True, speed=90):
        stop_k = 1 if sm < 15 else 3
        motor.reset()
        self.mLeft.position = 0
        self.mRight.position = 0
        encL = abs(self.mLeft.position)
        encR = abs(self.mRight.position)
        min_speed = 40
        max_speed = speed
        speed = min_speed
        startL = encL
        startR = encR
        nB = int(self.sm2cpr(abs(sm)) + (startL + startR)/2)
        while (encL + encR >> 1 <= nB):
            encL = abs(self.mLeft.position)
            encR = abs(self.mRight.position)
            err = (encL) - (encR)
            p_koef = 1  # 1.5
            speedL = (speed - err * p_koef)
            speedR = (speed + err * p_koef)
            # print(speedL)
            print(speed)
            speed += 5 if speed < max_speed and (encL + encR)//2 < (nB // 4 * stop_k) else 0
            speed -= 10 if speed > min_speed - 10 and (encL + encR)//2 > (nB // 4 * stop_k) else 0
            self.motors(speedL * self.sign(sm),speedR * self.sign(sm))
        if stop:
            self.motors(0, 0)
            self.stop()

    def turnEnc(self, degrees, type=1, speed=150, stop=True):  # 50
        motor.reset()
        self.mLeft.position = 0
        self.mRight.position = 0
        startL = self.mLeft.position
        startR = self.mRight.position
        if type == 1:
            speed = 300  # +50 100 350 150
            motor_degree = abs(
                int(self.sm2cpr(self.robot['track'] * math.pi * degrees / 360)))
            print(motor_degree)
            p_koef = 2  # 1 2 1
            startL = abs(self.mLeft.position)
            startR = abs(self.mRight.position)
            encL = startL
            encR = startR
            nL = motor_degree + startL
            nR = motor_degree + startR
            while (encL < nL or encR < nR):
                encL = abs(self.mLeft.position)
                encR = abs(self.mRight.position)
                errL = nL - encL
                errR = nR - encR
                errB = (encL - startL) - (encR - startR)
                pL = errL * p_koef
                pR = errR * p_koef
                pB = errB * 5  # 0.5
                speedL = speed + pL - pB
                speedR = speed + pR + pB
                speed -= 10 if speed > 200 else 0
                self.mLeft.run_forever(speed_sp=-speedL * self.sign(degree))
                self.mRight.run_forever(speed_sp=-speedR * self.sign(degrees))
        elif type == 'R':
            speed = 150 if self.sign(degrees) > 0 else 300
            motor_degree = abs(
                int(self.sm2cpr(self.robot['track']) * (math.pi * degrees / 180)))
            print(motor_degree)
            p_koef = 0.7
            encR = abs(self.mRight.position)
            startR = encR
            print(startR)
            nR = startR + motor_degree
            while (encR < nR):
                encR = abs(self.mRight.position)
                errR = nR - encR
                pR = errR * p_koef
                speedR = speed + pR
                speed -= 10 if speed > 200 else 0
                speedR = 900 if speedR > 900 else speedR
                speedR = -900 if speedR < -900 else speedR
                self.mRight.run_forever(speed_sp=-speedR * self.sign(degrees))
                self.mLeft.run_forever(speed_sp=0)
        elif type == 'L':
            speed = 150 if self.sign(degrees) < 0 else 300
            motor_degree = abs(
                int(self.sm2cpr(self.robot['track']) * (math.pi * degrees / 180)))
            print(motor_degree)
            p_koef = 0.7
            encL = abs(self.mLeft.position)
            startL = encL
            nL = startL + motor_degree
            while (encL < nL):
                encL = abs(self.mLeft.position)
                errL = nL - encL
                pL = errL * p_koef
                speedL = speed + pL
                speed -= 10 if speed > 200 else 0
                speedL = 900 if speedL > 900 else speedL
                speedL = -900 if speedL < -900 else speedL
                self.mLeft.run_forever(speed_sp=-speedL * self.sign(degrees))
                self.mRight.run_forever(speed_sp=0)
            if stop:
                self.stop()

    def line_two_black(self, speed, p_koef, d_koef=0, stop=True):
        left = leftLine.value()//line_k
        right = rightLine.value()//line_k
        last_err = left - right
        while left > self.left['black'] or right > self.right['black']:
            left = leftLine.value()//line_k
            right = rightLine.value()//line_k
            err = left - right
            P = err * p_koef
            D = (err - last_err) * d_koef
            speedL = speed + P + D
            speedR = speed - P - D
            last_err = err
            self.motors(speedL, speedR)
        if stop:
            self.stop()

    def line_one_black(self, speed, p_koef, d_koef=0, stop=True):
        left = leftLine.value()//line_k
        right = rightLine.value()//line_k
        last_err = left - right
        while left > self.left['gray'] and right > self.right['gray']:
            left = leftLine.value()//line_k
            right = rightLine.value()//line_k
            err = left - right
            P = err * p_koef
            D = (err - last_err) * d_koef
            speedL = speed + P + D
            speedR = speed - P - D
            last_err = err
            self.motors(speedL, speedR)
        if stop:
            self.stop()

    def line_two_enc(self, sm, speed, p_koef, d_koef=0, stop=True):
        self.mLeft.position = 0
        self.mRight.position = 0
        left = leftLine.value()//line_k
        right = rightLine.value()//line_k
        last_err = left - right
        startL = abs(self.mLeft.position)
        startR = self.mRight.position
        while abs(self.mLeft.position) <= abs(self.sm2cpr(sm)) + startL:
            left = leftLine.value()//line_k
            right = rightLine.value()//line_k
            err = left - right
            P = err * p_koef
            D = (err - last_err) * d_koef
            speedL = speed + P + D
            speedR = speed - P - D
            self.motors(speedL, speedR)
            last_err = err
        if stop:
            self.stop()

    def moveLine(self, speed=50, stop=True):
        self.mLeft.position = 0
        self.mRight.position = 0
        encL = abs(self.mLeft.position)
        encR = abs(self.mRight.position)
        startL = encL
        startR = encR
        while leftLine.value()//line_k > self.left['black'] and rightLine.value()//line_k > self.right['black']:
            encL = abs(self.mLeft.position)
            encR = abs(self.mRight.position)
            err = (encL - startL) - (encR - startR)
            p_koef = 1  # 2
            speedL = speed - err * p_koef
            speedR = speed + err * p_koef
            self.motors(speedL, speedR)
        if stop:
            self.stop()

    def alignLine(self, speed=40):
        align_speed = 200  # 150
        TIMES_TO_ALIGN = 3
        self.mLeft.position = 0
        self.mRight.position = 0
        encL = abs(self.mLeft.position)
        encR = abs(self.mRight.position)
        startL = encL
        startR = encR
        while leftLine.value()//line_k > self.left['gray'] and rightLine.value()//line_k > self.right['gray']:
            encL = abs(self.mLeft.position)
            encR = abs(self.mRight.position)
            err = (encL - startL) - (encR - startR)
            p_koef = 1
            speedL = speed - err * p_koef
            speedR = speed + err * p_koef
            self.motors(speedL, speedR)
        self.stop()
        # need to rewrite

    def calibrate_sensors(self):
        left_sensor = []
        right_sensor = []
        Sound.beep().wait()
        sleep(0.2)
        Sound.beep().wait()
        while True:
            left_sensor.append(leftLine.value()//10)
            right_sensor.append(rightLine.value()//10)
            if ev3_button.btn.any():
                break
        Sound.beep().wait()
        sleep(0.2)
        Sound.beep().wait()
        file1 = open('left_sensor.txt', 'w')
        file2 = open('right_sensor.txt', 'w')
        file1.write(str(min(left_sensor)) + '\n')
        file2.write(str(min(right_sensor)) + '\n')
        file1.write(str(max(left_sensor)) + '\n')
        file2.write(str(max(right_sensor)) + '\n')
        file1.write(str((max(left_sensor) + min(left_sensor)) // 2) + '\n')
        file2.write(str((max(right_sensor) + min(right_sensor)) // 2) + '\n')
        file1.close()
        file2.close()

    def line_right_norm(self, speed, stop=False, p_koef=1, d_koef=0):
        last_error = 0
        while leftLine.value()//line_k > 35:
            val = rightLine.value()//line_k
            err = -(val - 40)
            P = err * p_koef
            D = (err - last_error) * d_koef
            speedL = speed + P + D
            speedR = speed - P - D
            self.motors(speedL, speedR)
            last_error = err
        if stop:
            self.stop()

    def line_right_inversia(self, speed, stop=False, p_koef=-1, d_koef=0):
        last_error = 0
        while leftLine.value()//line_k < 35:
            val = rightLine.value()//line_k
            err = -(val - 40)
            P = err * p_koef
            D = (err - last_error) * d_koef
            speedL = speed + P + D
            speedR = speed - P - D
            self.motors(speedL, speedR)
            last_error = err
        if stop:
            self.stop()

    def align_norm(self):
        Sound.beep().wait()
        self.mLeft.run_forever(speed_sp=0)
        while rightLine.value()//line_k > self.left['gray']:
            self.mRight.run_forever(speed_sp=200)
        motor.stop()
        Sound.beep().wait()
    def align_inversia(self):
        self.mLeft.run_forever(speed_sp=0)
        while rightLine.value()//line_k < 35:
            self.mRight.run_forever(speed_sp=200)
        motor.stop()


class Claw:
    def __init__(self):
        self.claw = MediumMotor('outA')
        self.lift = LargeMotor('outD')
        self.claw.reset()
        self.lift.reset()

    def calibrate(self):
        self.claw.run_forever(speed_sp=-300)
        sleep(3)
        self.claw.stop()
        self.claw.run_to_rel_pos(position_sp=250, speed_sp=900)

    def lift_up(self):
        self.lift.run_forever(speed_sp=300)

    def lift_down(self):
        self.lift.run_forever(speed_sp=-300)

    def catch_object(self):
        self.claw.run_forever(speed_sp=-500)
        sleep(1)
        self.claw.run_forever(speed_sp=500)
        sleep(0.5)
        self.claw.stop()
        motor.moveEnc(sm=2, stop=True)
        self.claw.run_forever(speed_sp=-500)
        sleep(0.7)
        self.lift.run_forever(speed_sp=700)

    def stop(self):
        self.claw.stop()
        self.lift.stop()

    def catch(self):
        self.claw.run_forever(speed_sp=-500)
        sleep(1)


class Kicker:
    def __init__(self):
        self.kicker = MediumMotor('outA')
        self.kicker.reset()
        self.kicker.run_forever(speed_sp=400)
        sleep(0.1)

    def kick(self):
        self.kicker.run_forever(speed_sp=-900)
        sleep(0.3)
        self.kicker.run_forever(speed_sp=900)


class ColorSensorHT():
    def __init__(self):
        self.color_sensor = Sensor(address='i2c-legoev33:i2c1')
        self.color_sensor.mode = "RGB"

    def getRGB(self):
        R = self.color_sensor.value(0)
        G = self.color_sensor.value(1)
        B = self.color_sensor.value(2)
        rgb = [R, G, B]
        return rgb

    def getMinMax(self):
        arr = self.getRGB()
        value_min = min(arr)
        value_max = max(arr)
        arr = [value_min, value_max]
        return arr

    def getH(self):
        rgb = self.getRGB()
        R = rgb[0]
        G = rgb[1]
        B = rgb[2]
        minmax = self.getMinMax()
        min_v = minmax[0]
        max_v = minmax[1]
        if min_v == max_v:
            H = 0
        else:
            if max_v == R:
                if G >= B:
                    H = 60 * (G - B)/(max_v - min_v)
                else:
                    G = 60 * (G - B)/(max_v - min_v) + 360
            else:
                if max_v == G:
                    H = 60 * (B - R)/(max_v - min_v) + 120
                else:
                    H = 60 * (R - G)/(max_v - min_v) + 240
        return H

    def getS(self):
        minmax = self.getMinMax()
        min_v = minmax[0]
        max_v = minmax[1]
        if max_v == 0:
            S = 0
        else:
            S = 1 - min_v / max_v
        return S

    def getV(self):
        minmax = self.getMinMax()
        V = minmax[1]
        return V


def Line_with_obstacle_Forward():
    speed = 10
    while leftLine.value()//line_k < motor.left['gray'] and rightLine.value()//line_k < motor.right['gray']:
        speed += 1
        motor.motors(speed, speed)
    motor.stop()
    motor.line_two_enc(speed=90, sm=80, p_koef=0.7, d_koef=0)
    motor.line_two_enc(speed=55, sm=40, p_koef=1.4, d_koef=0)
    motor.align_norm()
    motor.turnEnc(degrees=-45, type="R")
    motor.moveEnc(sm = 20)
    sleep(0.2)
    motor.turnEnc(degrees = 50,type = "L")
    
    motor.moveEnc(sm = 37)
    motor.turnEnc(degrees = 50,type = "L")
    motor.moveLine(speed = 50)
    #motor.alignLine()
    #motor.turnEnc(degrees = -90,type = "R")
    #motor.moveEnc(sm = 4)
    #sleep(0.5)
    motor.line_two_enc(speed=50, p_koef=1.3, d_koef=0,sm=90)
    motor.line_two_enc(speed = 80,sm = 60,p_koef = 0.7,d_koef = 0)
    #motor.line_two_enc(speed = 90,sm = 60,p_koef = 0.5,d_koef = 0)
    motor.line_two_black(speed = 50,p_koef = 1.3,d_koef = 0)
    


def Line_with_obstacle_Backward():
    motor.line_two_enc(speed=70, sm=90, p_koef=2.2, d_koef=0)
    motor.line_two_enc(speed=90, sm=70, p_koef=0.5, d_koef=0)
    motor.line_two_enc(speed=80, sm=45, p_koef=2.65, d_koef=0)
    motor.turnEnc(degrees=90, type="L")
    motor.moveEnc(sm=20)
    motor.turnEnc(degrees=-95, type="R")
    motor.moveEnc(sm=50)
    motor.turnEnc(degrees=-90, type="R")
    motor.moveLine(speed=50)
    motor.turnEnc(degrees=90, type="L")
    motor.moveEnc(-10)
    sleep(0.5)
    motor.line_two_enc(speed=80, sm=80, p_koef=2.8, d_koef=0)
    motor.line_two_black(speed=80, p_koef=0.5, d_koef=0)
    motor.moveEnc(sm=10)


def InversiaForward():
    sp_inv = 80
    sp_str = 65
    motor.line_right_norm(speed=50)
    motor.moveEnc(sm=12)  # 12
    motor.turnEnc(degrees=90, type="L")
    motor.align_norm()
    motor.line_right_norm(speed=sp_str)
    motor.line_right_inversia(speed=sp_inv)
    motor.line_right_norm(speed=sp_str, stop=True)
    motor.moveEnc(sm=10, stop=True)
    motor.turnEnc(degrees=-90, type="R")
    motor.moveEnc(sm=12, stop=True)  # 10
    motor.turnEnc(degrees=-90, type="R")
    motor.moveEnc(5)
    motor.align_inversia()
    motor.line_right_inversia(speed=sp_str)
    motor.line_right_norm(speed=sp_inv)
    motor.line_right_inversia(speed=sp_str, stop=True)
    motor.moveEnc(sm=10, stop=True)
    motor.turnEnc(degrees=90, type="L")
    motor.moveEnc(sm=8, stop=True)  # 10
    motor.turnEnc(degrees=90, type="L")
    motor.moveEnc(sm=5, stop=False)
    motor.align_norm()
    motor.line_right_norm(speed=sp_str, stop=True)
    motor.moveEnc(sm=10, stop=True)  # 12
    motor.turnEnc(degrees=-90, type="R")
    motor.align_norm()
    motor.line_right_norm(speed=50, stop=True)


def InversiaBackward():
    sp_inv = 80
    sp_str = 65
    motor.line_right_norm(speed=70)
    motor.moveEnc(sm=10, stop=True)  # 10
    motor.turnEnc(degrees=90, type="L")
    motor.align_norm()
    motor.line_right_norm(speed=sp_str)
    motor.moveEnc(sm=10, stop=True)
    motor.turnEnc(degrees=-90, type="R")
    motor.moveEnc(sm=10, stop=True)  # 10
    motor.turnEnc(degrees=-90, type="R")
    # motor.moveEnc(5)
    motor.align_inversia()
    motor.line_right_inversia(speed=sp_str)
    motor.line_right_norm(speed=sp_inv)
    motor.line_right_inversia(speed=sp_str, stop=True)
    motor.moveEnc(sm=10, stop=True)
    motor.turnEnc(degrees=90, type="L")
    motor.moveEnc(sm=10, stop=True)  # 10
    motor.turnEnc(degrees=90, type="L")
    # motor.moveEnc(5)
    motor.align_norm()
    motor.line_right_norm(speed=sp_str)
    motor.line_right_inversia(speed=sp_inv)
    motor.line_right_norm(speed=sp_str, stop=True)
    motor.moveEnc(sm=10, stop=True)
    motor.turnEnc(degrees=-90, type="R")
    motor.align_norm()
    motor.line_right_norm(speed=sp_str, stop=True)


def Kegeling():
    sp = 300
    fsp = 70
    degr = 93
    d = 1
    motor.turnEnc(degrees=-27, type="L")
    claw.lift_down()
    motor.moveEnc(sm=40, stop=False)
    motor.moveLine(speed=fsp, stop=False)
    motor.moveEnc(sm=15, stop=True)
    motor.turnEnc(degrees=degr, type="R")
    motor.moveEnc(sm=15, stop=False)
    motor.moveLine(speed=fsp, stop=False)
    motor.moveEnc(sm=15, stop=True)
    motor.turnEnc(degrees=degr, type="R")
    motor.moveEnc(sm=15, stop=False)
    motor.moveLine(speed=fsp, stop=False)
    motor.moveEnc(sm=15, stop=True)
    motor.turnEnc(degrees=degr, type="R")
    motor.moveEnc(sm=15, stop=False)
    motor.moveLine(speed=fsp, stop=False)
    motor.moveEnc(sm=15, stop=True)
    sleep(0.5)
    motor.turnEnc(degrees=130, type="R")
    motor.moveEnc(sm=20, stop=True)
    motor.moveEnc(sm=17, speed=55, stop=True)
    claw.catch_object()
    sleep(1.5)
    motor.turnEnc(degrees=150, type="L")
    # motor.moveEnc(10)
    sleep(0.1)
    #motor.turnEnc(degrees = 94,type = "R")
    motor.moveLine(speed=50)
    motor.moveEnc(sm=10, stop=False)
    motor.moveLine(speed=50, stop=True)
    # motor.alignLine()
    motor.moveEnc(5)
    motor.align_norm()


def MoveForward(sm=30):
    wall = 6
    if rightUS.value()//10 < 10:  # 9 5 and rightUS.value()//10 > wall
        Sound.beep().wait()
        us = rightUS.value()//10
        motor.mRight.stop()
        motor.mLeft.run_forever(speed_sp=-200)
        while rightUS.value()//10 > wall:
            us = rightUS.value()//10

        motor.stop()
    min_speed = 20
    max_speed = 60
    speed = min_speed  # 40 50 45
    # wall = 5 # 4 4
    motor.reset()
    motor_degree = int(motor.sm2cpr(sm))
    motor_threshold = motor.sm2cpr(8)
    startL = abs(motor.mLeft.position)
    startR = abs(motor.mRight.position)
    encL = abs(motor.mLeft.position)
    encR = abs(motor.mRight.position)
    Flag = False
    while int((encL + encR)/2) < motor_degree and frontUS.value()//10 > 8:
        encL = abs(motor.mLeft.position)
        encR = abs(motor.mRight.position)
        usR = int(rightUS.value())//10
        usL = int(leftUS.value())//10
        #print(str(usR) + "_" + str(usL))
        p_koef = 6  # 4
        k = 1
        tr = 11
        if usR < tr * k and usL < tr * k:  # 6 6
            # led.led_off()
            # print(1)
            err = usR - usL
            P = err * p_koef  # 10 15 10 5 7.5
            speedL = speed + P
            speedR = speed - P
        elif usR > tr * k and usL < tr * k:
            # print(2)
            # led.red("left")
            err = wall - usL
            P = err * p_koef  # 10 15 10 5 7.5
            speedL = speed + P
            speedR = speed - P
        elif usR < tr * k and usL > tr * k:
            # print(3)
            # led.red("right")
            err = usR - wall
            P = err * p_koef  # 10 15 10 5 7.5
            speedL = speed + P
            speedR = speed - P
        else:
            # print(4)
            # led.red("both")
            speedL = speed
            speedR = speed
        #speed += 1 if speed < max_speed else 0
        speed += 2 if speed < max_speed and (encL + encR) / \
            2 < (motor_degree/2) else 0
        speed -= 4 if speed > min_speed + \
            5 and (encL + encR)/2 > (motor_degree//4 * 3) else 0
        # print(speed)
        if speedL > 100:
            speedL = 100
        if speedL < -100:
            speedL = -100
        if speedR > 100:
            speedR = 100
        if speedR < -100:
            speedR = -100
        motor.mLeft.run_forever(speed_sp=-speedL * 9)
        motor.mRight.run_forever(speed_sp=speedR * 9)
        # sleep(0.001)
    motor.stop()
    # sleep(0.1)


def AlignFront(sm):
    start = frontUS.value()//10
    wall = sm
    us = frontUS.value()//10
    integral = 0
    motor.stop()
    while wall != us:
        us = frontUS.value()//10
        err = us - wall
        P = err * 15
        I = integral * 1
        speed = P + I
        if speed > 900:
            speed = 900
        if speed < -900:
            speed = -900
        motor.mLeft.run_forever(
            speed_sp=speed * motor.robot['leftMotor_reverse'])
        motor.mRight.run_forever(
            speed_sp=speed * motor.robot['rightMotor_reverse'])
        integral = integral + err
    motor.stop()


def Maze():
    left_turn_counter = 0
    while True:
        if frontUS.value()//10 < 20:
            AlignFront(3)
        if rightUS.value()//10 > 20 and rightUS.value()//10 != 255:
            #motor.moveEnc(speed = 40,sm = 1.5,stop = True)
            # sleep(0.01)
            motor.stop()
            motor.turnEnc(degrees=90, type=1, stop=True)
            # sleep(0.01)
            # motor.moveEnc(speed = 40,sm = -5,stop = True) #3
            # sleep(0.3)
            MoveForward()
            # MoveForward(36)
        elif frontUS.value()//10 > 20:
            if left_turn_counter == -1 and rightUS.value()//10 != 5:  # and rightUS.value()//10 != 5
                Sound.beep().wait()
                left_turn_counter = 0
                motor.motors(-80, -80)
                sleep(0.6)
                motor.stop()
                MoveForward(4)
            MoveForward()
            left_turn_counter = 0
        else:
            d = 0.05
            sleep(d)
            motor.stop()
            motor.turnEnc(degrees=-90, type=1, stop=True)  # 88
            sleep(d)
            motor.moveEnc(speed=50, sm=-3, stop=True)
            # sleep(0.05)
            left_turn_counter += 1
            if left_turn_counter == -1:
                Sound.beep().wait()
                left_turn_counter = 0
                motor.motors(-80, -80)
                sleep(0.6)
                motor.stop()
                MoveForward(4)


line_k = 10
motor = Motor()
lcd = lcd()
led = leds()
claw = Claw()
led.red("both")
lcd.lcd_clear()
print("program started")
ev3_button = button()

rightUS = UltrasonicSensor('in3')
leftUS = UltrasonicSensor('in2')
frontUS = UltrasonicSensor('in4')

mux2 = LegoPort(address='i2c-legoev33:i2c8:mux2')
mux2.mode = 'analog'
mux2.set_device = 'lego-nxt-light'
mux3 = LegoPort(address='i2c-legoev33:i2c8:mux3')
mux3.mode = 'analog'
mux3.set_device = 'lego-nxt-light'

sleep(1)
leftLine = LightSensor('i2c-legoev33:i2c8:mux2')
rightLine = LightSensor('i2c-legoev33:i2c8:mux3')
# claw.lift_up()
Sound.beep().wait()
#motor.turnEnc(-90,type = "R")
claw.lift_up()
ev3_button.waitButtons()
#motor.moveEnc(sm = 20)

Line_with_obstacle_Forward()
#motor.turnEnc(type = "R",degrees = -45)
motor.stop()
claw.stop()
claw.reset()

