from Jetson import GPIO
import time 
import numpy as np
import os
import warnings

class  Stepper():
    def __init__(self,calibration_range = 0, delay = 1e-16,max_steps = 190000):
        # 450 step = 1 mm
        # 192750 steps from the starting point
        self.max_steps = max_steps
        self.direction = 'forward'
        self.all_steps = 0
        self._1_mm = 400
        
        self.out1 = 38
        self.out2 = 35
        self.out3 = 37
        self.out4 = 36
        
        self.i=0
        self.positive=0
        self.negative=0
        self.y=0
        self.delay = delay
        self.setup(calibration_range)
        print('after calibration : relative distance from initial point is {}'.format(self.all_steps))
        if os.path.exists('zero_memory'):
            print('reading position memory file ...')
            self.all_steps = self.recall_memory()
            print('camera is at distance {} (mm) from starting point'.format(self.all_steps))
            #self.drive(0)
            print('camera is at distance {} (mm) from starting point'.format(self.all_steps))
        else:
            self.drive(0)
            print('camera is at distance {} (mm) from starting point'.format(self.all_steps))
            self.drive(0)
        self.drive(0)
        
    def setup(self, calibration_range):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.out1,GPIO.OUT)
        GPIO.setup(self.out2,GPIO.OUT)
        GPIO.setup(self.out3,GPIO.OUT)
        GPIO.setup(self.out4,GPIO.OUT)
        warnings.warn('motor is calibrating')
        self.drive(calibration_range)
        time.sleep(0.5)
        self.drive(-1*calibration_range)
        time.sleep(0.5)
        self.all_low()

    def all_low(self):
        GPIO.output(self.out1,GPIO.LOW)
        GPIO.output(self.out2,GPIO.LOW)
        GPIO.output(self.out3,GPIO.LOW)
        GPIO.output(self.out4,GPIO.LOW)
        time.sleep(self.delay)
        
    def h1l3(self):
        GPIO.output(self.out1,GPIO.HIGH)
        GPIO.output(self.out2,GPIO.LOW)
        GPIO.output(self.out3,GPIO.LOW)
        GPIO.output(self.out4,GPIO.LOW)
        time.sleep(self.delay)
        
    def h2l2(self):
        GPIO.output(self.out1,GPIO.HIGH)
        GPIO.output(self.out2,GPIO.HIGH)
        GPIO.output(self.out3,GPIO.LOW)
        GPIO.output(self.out4,GPIO.LOW)
        time.sleep(self.delay)
        
    def l1h1l2(self):
        GPIO.output(self.out1,GPIO.LOW)
        GPIO.output(self.out2,GPIO.HIGH)
        GPIO.output(self.out3,GPIO.LOW)
        GPIO.output(self.out4,GPIO.LOW)
        time.sleep(self.delay)
        
    def l1h2l1(self):
        GPIO.output(self.out1,GPIO.LOW)
        GPIO.output(self.out2,GPIO.HIGH)
        GPIO.output(self.out3,GPIO.HIGH)
        GPIO.output(self.out4,GPIO.LOW)
        time.sleep(self.delay)
        
    def l2h1l1(self):
        GPIO.output(self.out1,GPIO.LOW)
        GPIO.output(self.out2,GPIO.LOW)
        GPIO.output(self.out3,GPIO.HIGH)
        GPIO.output(self.out4,GPIO.LOW)
        time.sleep(self.delay)
        
    def l2h2(self):
        GPIO.output(self.out1,GPIO.LOW)
        GPIO.output(self.out2,GPIO.LOW)
        GPIO.output(self.out3,GPIO.HIGH)
        GPIO.output(self.out4,GPIO.HIGH)    
        time.sleep(self.delay)
        
    def l3h1(self):
        GPIO.output(self.out1,GPIO.LOW)
        GPIO.output(self.out2,GPIO.LOW)
        GPIO.output(self.out3,GPIO.LOW)
        GPIO.output(self.out4,GPIO.HIGH)
        time.sleep(self.delay)
        
    def h1l2h1(self):
        GPIO.output(self.out1,GPIO.HIGH)
        GPIO.output(self.out2,GPIO.LOW)
        GPIO.output(self.out3,GPIO.LOW)
        GPIO.output(self.out4,GPIO.HIGH)
        time.sleep(self.delay)
    
    def feed_forward(self,i):
        if i==0:
            self.h1l3()
        elif i==1:
            self.h2l2()
        elif i==2:  
            self.l1h1l2()
        elif i==3:  
            self.l1h2l1()  
        elif i==4:
            self.l2h1l1()
        elif i==5:
            self.l2h2()
        elif i==6:    
            self.l3h1()
        elif i==7:    
            self.h1l2h1()
 
    def check_position(self):
        pass 
 
    def drive(self, dist):
        tmp = self.all_steps
        try:
            self.all_steps = self.recall_memory()
        except: 
            pass
        warnings.warn('camera is moving')
        Number_of_steps = self.dist2steps(dist)
        self.all_steps += Number_of_steps
        self.distance = (tmp+Number_of_steps)/self._1_mm
        print('camera is moving to {} mm form the starting point\n \
                                Number of steps = {}\n \
                                past sum of steps = {} \n \
                                current sum of steps = {}\n \
                                '.format((tmp+Number_of_steps)/self._1_mm, Number_of_steps, tmp,self.all_steps))
        
        if abs((tmp+Number_of_steps)/self._1_mm) > self.max_steps/self._1_mm or  \
            (tmp/self._1_mm) >= 0:  
            print('abs((tmp+Number_of_steps)/self._1_mm) : ',abs((tmp+Number_of_steps)/self._1_mm))
            print('self.max_steps/self._1_mm: ', self.max_steps/self._1_mm)
            print('(tmp/self._1_mm): ',(tmp/self._1_mm), 'tmp: ', tmp)
            self.all_low()
            return False
        self.zero_point_memory(self.all_steps)
        
        if Number_of_steps>0:
            for y in range(Number_of_steps,0,-1):
                if self.negative==1:
                    if self.i==7:
                        self.i=0
                    else:
                        self.i=self.i+1
                    y=y+2
                    self.negative=0
                self.positive=1
                self.feed_forward(self.i)
                if self.i==7:
                    self.i=0
                    continue
                self.i=self.i+1
        elif Number_of_steps<0:
            Number_of_steps = Number_of_steps*-1
            for y in range(Number_of_steps,0,-1):
                if self.positive==1:
                    if self.i==0:
                        self.i=7
                    else:
                        self.i=self.i-1
                    y=y+3
                    self.positive=0
                self.negative=1
                self.feed_forward(self.i)
                if self.i==0:
                    self.i=7
                    continue
                self.i=self.i-1
        print('camera is at {} mm form the starting point'.format(self.all_steps/self._1_mm))
        self.all_low()
        return True

    def zero_point_memory(self, steps):
        with open('zero_memory','w') as self.zp:
            steps = str(steps)+'\n'
            self.zp.write(steps)
            
    def recall_memory(self):
        with open('zero_memory','r') as zp:
            p = int(zp.readline())
            return p
            
    def dist2steps(self, dist):
        # 1mm = 400 steps
        # dist in mm
        if dist*self._1_mm > self.max_steps:
            raise ValueError('the distance should be less than {} \n \
                                max_steps = {} \n \
                                input_steps = {}\n \
                                1 mm is equivalent to {} steps\n \
                                input distance is {}\n'.format(self.max_steps/self._1_mm,self.max_steps,dist*self._1_mm, self._1_mm, dist))
            return
        return dist*self._1_mm 
    

if __name__ == '__main__':
#     motor = Stepper(Number_of_steps=600)
#     motor.drive()
    try:
        i = 0
        motor = Stepper()
        direction = 'backward'# backward forward
        while True:
            i+=1
            displacement = 1 #mm
            if direction is 'forward': 
                displacement *= -1
            else:
                displacement *= 1
                
            motor_go = motor.drive(displacement)
            if not motor_go: break
            GPIO.cleanup
            time.sleep(0.010)
            #motor.drive(10)
        motor.zp.close()
        GPIO.cleanup()
        motor.zp.close()
            
            
    except KeyboardInterrupt:
        GPIO.cleanup()
        motor.zp.close()
        

            