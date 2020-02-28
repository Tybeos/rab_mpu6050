from OpenGL.GL import *
from OpenGL.GLU import *

import pygame
from pygame.locals import *

import smbus
import math
import time

class mpu_6050:
    
    """
    Must be enable SPI, I2C and OpenGL
    
    For more info check datasheet 
    
    # Pre-defined accelerometer ranges
    range: +-2 G  sensitivity: 16384 register address: 0x00 
    range: +-4 G  sensitivity: 8192  register address: 0x08
    range: +-8 G  sensitivity: 4096  register address: 0x10
    range: +-16 G sensitivity: 2048  register address: 0x18
    
    # Pre-defined gyroscope ranges
    range: +-250 deg  sensitivity: 131  register address: 0x00 
    range: +-500 deg  sensitivity: 65.5 register address: 0x08
    range: +-1000 deg sensitivity: 32.8 register address: 0x10
    range: +-2000 deg sensitivity: 16.4 register address: 0x18 
    """
    #I2C bus
    bus = smbus.SMBus(1)
    
    # Address
    ADDRESS = 0x68
     
    # Wake up register address
    PWR_MGMT_1 = 0x6B
    
    # Choose your range
    ACCEL_RANGE = 0x00
    GYRO_RANGE = 0x08
    
    # Registers for config
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1b
    
    #Set sensitivity by range
    GYR_SENSITIVITY = 65.5
    ACC_SENSITIVITY = 16384.0
    
    # Variables
    def __init__ (self):
        self.deltatime = 1
        self.gx = 1
        self.gy = 1
        self.gz = 1
        self.gyr_xout = 1
        self.gyr_yout = 1
        self.gyr_zout = 1
        self.acc_xout = 1
        self.acc_yout = 1
        self.acc_zout = 1
        self.gyrXoffs = 1
        self.gyrYoffs = 1
        self.gyrZoffs = 1
        
    def setup_mpu_6050(self):
        
        print ("Setup\n")
        # Wake up
        self.bus.write_byte_data(self.ADDRESS, self.PWR_MGMT_1, 0x00)
         
        # Low pass filter samples, 1khz sample rate
        self.bus.write_byte_data(self.ADDRESS,  0x1a, 0x01)
        
        # Reset settings accelerometer range
        self.bus.write_byte_data(self.ADDRESS, self.ACCEL_CONFIG, 0x00)
        # Setting accelerometer range
        self.bus.write_byte_data(self.ADDRESS, self.ACCEL_CONFIG, self.ACCEL_RANGE)
        
        # Reset settings gyroscope range
        self.bus.write_byte_data(self.ADDRESS, self.GYRO_CONFIG, 0x00)
        # Setting gyroscope range
        self.bus.write_byte_data(self.ADDRESS, self.GYRO_CONFIG, self.GYRO_RANGE)
        
        # Calculating offset from 500 samples
        # Don't move with the module during measurement
        print ("Starting calibration\n")
        
        num = 500
        xsum = ysum = zsum = 0.0
        
        for i in range(num):
            xsum += self.read_data(0x43)
            ysum += self.read_data(0x45)
            zsum += self.read_data(0x47)
        
        self.gyrXoffs = xsum / num
        self.gyrYoffs = ysum / num
        self.gyrZoffs = zsum / num
       
        print ("Done\n")
    
    def read_data(self, REG):
        
        high = self.bus.read_byte_data(self.ADDRESS, REG)
        low = self.bus.read_byte_data(self.ADDRESS, REG + 1)

        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value
        
    def get_data(self):
        
        """
        time.clock_gettime Return time
        time.CLOCK_MONOTONIC_RAW provides access to a raw hardware-based time that is not subject to NTP adjustments.
        Availability: Linux 2.6.28 and newer, macOS 10.12 and newer.
        """
        timein = time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
        #Reading  gyroscope data
        self.gyr_xout = (self.read_data(0x43) - self.gyrZoffs) / self.GYR_SENSITIVITY
        self.gyr_yout = (self.read_data(0x45) - self.gyrZoffs) / self.GYR_SENSITIVITY
        self.gyr_zout = (self.read_data(0x47) - self.gyrZoffs) / self.GYR_SENSITIVITY
        #Reading  accelerometer data
        self.acc_xout = self.read_data(0x3b) / self.ACC_SENSITIVITY
        self.acc_yout = self.read_data(0x3d) / self.ACC_SENSITIVITY
        self.acc_zout = self.read_data(0x3f) / self.ACC_SENSITIVITY
        
        # Calculating Roll from accelerometer
        self.ax = math.degrees(math.atan2(self.acc_yout, math.hypot(self.acc_xout, self.acc_zout)))
        # Calculating Pitch from accelerometer
        self.ay = math.degrees(math.atan2(self.acc_xout, math.hypot(self.acc_yout, self.acc_zout)))
        
        # Calculating Roll from gyroscope
        self.gx = self.gx + self.gyr_xout * self.deltatime
        # Calculating Pitch from gyroscope
        self.gy = self.gy - self.gyr_yout * self.deltatime
        # Calculating Yaw from gyroscope
        self.gz = self.gz + self.gyr_zout * self.deltatime
        
        # Complementary filter, combinate data from gyroscope and accelerometer
        self.gx = self.gx * 0.96 + self.ax * 0.04
        self.gy = self.gy * 0.96 + self.ay * 0.04
        
        timeout = time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
        # Calculating delta time for gyroscope
        self.deltatime = timeout - timein

def resize(width, height):
    
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def draw(gx, gy, gz):
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
    glLoadIdentity()
    
    glTranslatef(0.0,0.0,-6.0)
    
    # Roll
    glRotatef(gx ,0.0,0.0,1.0)
    # Pitch
    glRotatef(-gy ,1.0,0.0,0.0)
    # Yaw
    #glRotatef(gz,0.0,1.0,0.0)
    
    verticies = ((1,-0.2,-1),(1,0.2,-1),(-1,0.2,-1),(-1,-0.2,-1),(1,-0.2,1),(1,0.2,1),(-1,-0.2,1),(-1,0.2,1))
    surfaces = ((0,1,2,3),(3,2,7,6),(6,7,5,4),(4,5,1,0),(1,5,7,2),(4,0,3,6))
    colours = ((1,1,0),(1,0,0),(1,0.6,0),(0,1,0),(0.5,0.5,0.5),(0.7,0.7,0.7))

    glBegin(GL_QUADS)
    
    for index, surface in enumerate(surfaces):
        glColor3fv(colours[index])
        for vertex in surface:
            glVertex3fv(verticies[vertex])
        
    glEnd()

def main():
    
    width = 600
    height = 500
    
    pygame.init()
    screen = pygame.display.set_mode((width, height), OPENGL|DOUBLEBUF)
    pygame.display.set_caption("MPU6050")
    
    resize(width, height)
    init()
    
    time.sleep(1)
    mpu = mpu_6050()
    mpu.setup_mpu_6050()
     
    while True:

        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()
            break
        
        mpu.get_data()
        draw(mpu.gx, mpu.gy, mpu.gz)

        pygame.display.flip()

        print_pitch = "pitch:" + str(round(mpu.gy,1))
        print_roll = "roll:" + str(round(mpu.gx,1)) + "\n" 
        print(print_pitch,print_roll)

if __name__ == '__main__':
    main()