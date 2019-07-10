import sys
import time
import logging
import traceback

default_logger = logging.getLogger(__name__)
default_logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

default_logger.addHandler(ch)

class ODriveInterfaceSimulator(object):
    encoder_cpr = 4096
    connected = False
    engaged = False

    right_axis_vel = 0 # units: encoder counts/s
    left_axis_vel  = 0
    right_axis_pos = 0 # go from 0 up to encoder_cpr-1
    left_axis_pos  = 0
    last_time_update = None
    
    
    def __init__(self, logger=None):
        self.logger = logger if logger else default_logger
        
    def update_time(self, curr_time):
        # provided so simulator can update position
        if last_time_update is None:
            last_time_update = curr_time
            return
        
        dt = curr_time - last_time_update
        left_axis_pos  = floor(left_axis_pos  + left_axis_vel *dt) % self.encoder_cpr
        right_axis_pos = floor(right_axis_pos + right_axis_vel*dt) % self.encoder_cpr
        last_time_update = curr_time
                                    
    def connect(self, port=None, right_axis=0, timeout=30):
        if self.connected:
            self.logger.info("Already connected. Simulating disc/reconnect.")
            
        self.encoder_cpr = 4096
        self.logger.info("Connected to simulated ODrive.")
        return True
        
    def disconnect(self):
        self.connected = False
        return True
                
    def calibrate(self):
        if not self.connected:
            self.logger.error("Not connected.")
            return False
        self.logger.info("Calibrated.")
        return True
        
    def preroll(self, wait=True, reverse=False):
        if not self.connected:
            self.logger.error("Not connected.")
            return False
        
        return True
                
    def ensure_prerolled(self):
        return True
    
    def engaged(self):
        return self.engaged
        
    def engage(self):
        if not self.connected:
            self.logger.error("Not connected.")
            return False
        self.engaged = True
        return True
        
    def release(self):
        if not self.connected:
            self.logger.error("Not connected.")
            return False
        self.engaged = False
        return True
    
    def drive(self, left_motor_val, right_motor_val):
        if not self.connnected:
            self.logger.error("Not connected.")
            return
        self.left_axis.controller.vel_setpoint = left_motor_val
        self.right_axis.controller.vel_setpoint = -right_motor_val
        
        return True
        
    def get_errors(self, clear=True):
        if not self.driver:
            return None
        return "Simulated ODrive, no errors."
        
    def left_vel_estimate(self):  return self.left_axis_vel
    def right_vel_estimate(self): return self.right_axis_vel
    def left_pos(self):           return self.left_axis_pos
    def right_pos(self):          return self.right_axis_pos
    
    def left_current(self):       return 0
    def right_current(self):      return 0
    