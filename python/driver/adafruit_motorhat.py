#!/usr/bin/python

from adafruit_pwm_servo_driver import PWM
import time

class AdafruitStepperMotor:
    MICROSTEPS = 8
    MICROSTEP_CURVE = [0, 50, 98, 142, 180, 212, 236, 250, 255]

    #MICROSTEPS = 16
    # a sinusoidal curve NOT LINEAR!
    #MICROSTEP_CURVE = [0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255]

    def __init__(self, controller, num, steps=200):
        self.MC = controller
        self.revsteps = steps
        self.motornum = num
        self.sec_per_step = 0.1
        self.steppingcounter = 0
        self.currentstep = 0

        num -= 1

        if (num == 0):
            self.PWMA = 8
            self.AIN2 = 9
            self.AIN1 = 10
            self.PWMB = 13
            self.BIN2 = 12
            self.BIN1 = 11
        elif (num == 1):
            self.PWMA = 2
            self.AIN2 = 3
            self.AIN1 = 4
            self.PWMB = 7
            self.BIN2 = 6
            self.BIN1 = 5
        else:
            raise NameError('MotorHAT Stepper must be between 1 and 2 inclusive')

    def set_speed(self, rpm):
        self.sec_per_step = 60.0 / (self.revsteps * rpm)
        self.steppingcounter = 0

    def one_step(self, dir, style):
        pwm_a = pwm_b = 255

        # first determine what sort of stepping procedure we're up to
        if (style == AdafruitMotorHAT.SINGLE):
            if ((self.currentstep/(self.MICROSTEPS/2)) % 2):
                # we're at an odd step, weird
                if (dir == AdafruitMotorHAT.FORWARD):
                    self.currentstep += self.MICROSTEPS/2
                else:
                    self.currentstep -= self.MICROSTEPS/2
            else:
                # go to next even step
                if (dir == AdafruitMotorHAT.FORWARD):
                    self.currentstep += self.MICROSTEPS
                else:
                    self.currentstep -= self.MICROSTEPS
        if (style == AdafruitMotorHAT.DOUBLE):
            if not (self.currentstep/(self.MICROSTEPS/2) % 2):
                # we're at an even step, weird
                if (dir == AdafruitMotorHAT.FORWARD):
                    self.currentstep += self.MICROSTEPS/2
                else:
                    self.currentstep -= self.MICROSTEPS/2
            else:
                # go to next odd step
                if (dir == AdafruitMotorHAT.FORWARD):
                    self.currentstep += self.MICROSTEPS
                else:
                    self.currentstep -= self.MICROSTEPS
        if (style == AdafruitMotorHAT.INTERLEAVE):
            if (dir == AdafruitMotorHAT.FORWARD):
                self.currentstep += self.MICROSTEPS/2
            else:
                self.currentstep -= self.MICROSTEPS/2

        if (style == AdafruitMotorHAT.MICROSTEP):
            if (dir == AdafruitMotorHAT.FORWARD):
                self.currentstep += 1
            else:
                self.currentstep -= 1

                # go to next 'step' and wrap around
                self.currentstep += self.MICROSTEPS * 4
                self.currentstep %= self.MICROSTEPS * 4

            pwm_a = pwm_b = 0
            if (self.currentstep >= 0) and (self.currentstep < self.MICROSTEPS):
                pwm_a = self.MICROSTEP_CURVE[self.MICROSTEPS - self.currentstep]
                pwm_b = self.MICROSTEP_CURVE[self.currentstep]
            elif (self.currentstep >= self.MICROSTEPS) and (self.currentstep < self.MICROSTEPS*2):
                pwm_a = self.MICROSTEP_CURVE[self.currentstep - self.MICROSTEPS]
                pwm_b = self.MICROSTEP_CURVE[self.MICROSTEPS*2 - self.currentstep]
            elif (self.currentstep >= self.MICROSTEPS*2) and (self.currentstep < self.MICROSTEPS*3):
                pwm_a = self.MICROSTEP_CURVE[self.MICROSTEPS*3 - self.currentstep]
                pwm_b = self.MICROSTEP_CURVE[self.currentstep - self.MICROSTEPS*2]
            elif (self.currentstep >= self.MICROSTEPS*3) and (self.currentstep < self.MICROSTEPS*4):
                pwm_a = self.MICROSTEP_CURVE[self.currentstep - self.MICROSTEPS*3]
                pwm_b = self.MICROSTEP_CURVE[self.MICROSTEPS*4 - self.currentstep]


        # go to next 'step' and wrap around
        self.currentstep += self.MICROSTEPS * 4
        self.currentstep %= self.MICROSTEPS * 4

        # only really used for microstepping, otherwise always on!
        self.MC._pwm.set_pwm(self.PWMA, 0, pwm_a*16)
        self.MC._pwm.set_pwm(self.PWMB, 0, pwm_b*16)

        # set up coil energizing!
        coils = [0, 0, 0, 0]

        if (style == AdafruitMotorHAT.MICROSTEP):
            if (self.currentstep >= 0) and (self.currentstep < self.MICROSTEPS):
                coils = [1, 1, 0, 0]
            elif (self.currentstep >= self.MICROSTEPS) and (self.currentstep < self.MICROSTEPS*2):
                coils = [0, 1, 1, 0]
            elif (self.currentstep >= self.MICROSTEPS*2) and (self.currentstep < self.MICROSTEPS*3):
                coils = [0, 0, 1, 1]
            elif (self.currentstep >= self.MICROSTEPS*3) and (self.currentstep < self.MICROSTEPS*4):
                coils = [1, 0, 0, 1]
        else:
            step2coils = [     [1, 0, 0, 0],
                [1, 1, 0, 0],
                [0, 1, 0, 0],
                [0, 1, 1, 0],
                [0, 0, 1, 0],
                [0, 0, 1, 1],
                [0, 0, 0, 1],
                [1, 0, 0, 1] ]
            coils = step2coils[self.currentstep/(self.MICROSTEPS/2)]

        #print "coils state = " + str(coils)
        self.MC.set_pin(self.AIN2, coils[0])
        self.MC.set_pin(self.BIN1, coils[1])
        self.MC.set_pin(self.AIN1, coils[2])
        self.MC.set_pin(self.BIN2, coils[3])

        return self.currentstep

    def step(self, steps, direction, stepstyle):
        s_per_s = self.sec_per_step
        lateststep = 0

        if (stepstyle == AdafruitMotorHAT.INTERLEAVE):
            s_per_s = s_per_s / 2.0
        if (stepstyle == AdafruitMotorHAT.MICROSTEP):
            s_per_s /= self.MICROSTEPS
            steps *= self.MICROSTEPS

        print s_per_s, " sec per step"

        for s in range(steps):
            lateststep = self.one_step(direction, stepstyle)
            time.sleep(s_per_s)

        if (stepstyle == AdafruitMotorHAT.MICROSTEP):
            # this is an edge case, if we are in between full steps, lets just keep going
            # so we end on a full step
            while (lateststep != 0) and (lateststep != self.MICROSTEPS):
                lateststep = self.one_step(dir, stepstyle)
                time.sleep(s_per_s)

class AdafruitDCMotor:
    def __init__(self, controller, num):
        self.MC = controller
        self.motornum = num
        pwm = in1 = in2 = 0

        if (num == 0):
                 pwm = 8
                 in2 = 9
                 in1 = 10
        elif (num == 1):
                 pwm = 13
                 in2 = 12
                 in1 = 11
        elif (num == 2):
                 pwm = 2
                 in2 = 3
                 in1 = 4
        elif (num == 3):
                 pwm = 7
                 in2 = 6
                 in1 = 5
        else:
            raise NameError('MotorHAT Motor must be between 1 and 4 inclusive')
        self.pwm_pin = pwm
        self.in1_pin = in1
        self.in2_pin = in2

    def run(self, command):
        if not self.MC:
            return
        if (command == AdafruitMotorHAT.FORWARD):
            self.MC.set_pin(self.in2_pin, 0)
            self.MC.set_pin(self.in1_pin, 1)
        if (command == AdafruitMotorHAT.BACKWARD):
            self.MC.set_pin(self.in1_pin, 0)
            self.MC.set_pin(self.in2_pin, 1)
        if (command == AdafruitMotorHAT.RELEASE):
            self.MC.set_pin(self.in1_pin, 0)
            self.MC.set_pin(self.in2_pin, 0)
    def set_speed(self, speed):
        if (speed < 0):
            speed = 0
        if (speed > 255):
            speed = 255
        self.MC._pwm.set_pwm(self.pwm_pin, 0, speed*16)

class AdafruitMotorHAT:
    FORWARD = 1
    BACKWARD = 2
    BRAKE = 3
    RELEASE = 4

    SINGLE = 1
    DOUBLE = 2
    INTERLEAVE = 3
    MICROSTEP = 4

    def __init__(self, addr = 0x60, freq = 1600):
        self._i2caddr = addr            # default addr on HAT
        self._frequency = freq        # default @1600Hz PWM freq
        self.motors = [ AdafruitDCMotor(self, m) for m in range(4) ]
        self.steppers = [ AdafruitStepperMotor(self, 1), AdafruitStepperMotor(self, 2) ]
        self._pwm =  PWM(addr, debug=False)
        self._pwm.set_pwm_freq(self._frequency)

    def set_pin(self, pin, value):
        if (pin < 0) or (pin > 15):
            raise NameError('PWM pin must be between 0 and 15 inclusive')
        if (value != 0) and (value != 1):
            raise NameError('Pin value must be 0 or 1!')
        if (value == 0):
            self._pwm.set_pwm(pin, 0, 4096)
        if (value == 1):
            self._pwm.set_pwm(pin, 4096, 0)

    def get_stepper(self, steps, num):
        if (num < 1) or (num > 2):
            raise NameError('MotorHAT Stepper must be between 1 and 2 inclusive')
        return self.steppers[num-1]

    def get_motor(self, num):
        if (num < 1) or (num > 4):
            raise NameError('MotorHAT Motor must be between 1 and 4 inclusive')
        return self.motors[num-1]
