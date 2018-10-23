import serial
from serial.serialutil import SerialException

import sys
import time
import logging

import odrive
from odrive.enums import *

default_logger = logging.getLogger(__name__)
default_logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

default_logger.addHandler(ch)

class ODriveInterfaceSerial(object):
    port = None
    
    def __init__(self, logger=None):
        self.logger = logger if logger else default_logger
        
    def connect(self, port):
        for retry in range(4):
            try:
                self.port = serial.Serial(port)
            except SerialException as e:
                if e.errno == 16: # [Errno 16] Device or resource busy
                    self.logger.debug("Busy. Retrying in 5s...")
                else:
                    raise
            time.sleep(5)
            if self.port:
                break
        if not self.port:
            self.logger.debug("Failed to connect to ODrive. Exiting.")
        self.port.timeout = 0.5
        
    def __del__(self):
        self.release()
        
    def setup(self):
        if not self.port:
            self.logger.error("Not connected.")
            return
            
        self.port.write('r vbus_voltage\n')
        voltage = self.port.read(100).strip()
        voltage = voltage if voltage else "unknown"
        self.logger.debug("Vbus: %s" % voltage)
        
        self.port.write('r vbus_voltage\n')
        voltage = self.port.read(100).strip()
        voltage = voltage if voltage else "unknown"
        self.logger.debug("Vbus: %s" % voltage)

        self.logger.debug("Calibrating left motor... (20 seconds)")
        self.port.write('w axis1.requested_state 3\n')
        time.sleep(20)

        self.logger.debug("Calibrating right motor... (20 seconds)")
        self.port.write('w axis0.requested_state 3\n')
        time.sleep(20)
        
    # https://github.com/madcowswe/ODrive/blob/1294ddff1dd0619e9f098ce12ca0936670a5b405/tools/odrive/enums.py
    # w axis1.requested_state 3 # calibrate
    # w axis1.requested_state 8 # closed loop control
    # w axis1.controller.config.control_mode 2 # velocity control
    # w axis1.controller.vel_setpoint 500
    # w axis1.requested_state 1
        
    def engage(self):
        if not self.port:
            self.logger.error("Not connected.")
            return
            
        self.logger.debug("Setting drive mode...")
        self.port.write('w axis0.requested_state 8\n')
        time.sleep(0.01)
        self.port.write('w axis1.requested_state 8\n')
        time.sleep(0.01)

        self.port.write('w axis0.controller.config.control_mode 2\n')
        time.sleep(0.01)
        self.port.write('w axis1.controller.config.control_mode 2\n')
        time.sleep(0.01)
        
        self.port.write('w axis0.controller.vel_setpoint 0\n')
        time.sleep(0.01)
        self.port.write('w axis1.controller.vel_setpoint 0\n')
        time.sleep(0.01)
        
    def release(self):
        self.logger.debug("Releasing.")
        self.port.write('w axis0.requested_state %d\n' % AXIS_STATE_IDLE)
        time.sleep(0.01)
        self.port.write('w axis1.requested_state %d\n' % AXIS_STATE_IDLE)
        time.sleep(0.01)
            
    def drive(self, left_motor_val, right_motor_val):
        if not self.port:
            self.logger.error("Not connected.")
            return
            
        self.port.write('w axis0.controller.vel_setpoint %d\n' % right_motor_val)
        time.sleep(0.01)
        self.port.write('w axis1.controller.vel_setpoint %d\n' % left_motor_val)
        time.sleep(0.01)


class ODriveInterfaceAPI(object):
    driver = None
    encoder_cpr = 4096
    right_axis = None
    left_axis = None
    connected = False
    
    def __init__(self, logger=None):
        self.logger = logger if logger else default_logger
                
    def __del__(self):
        self.disconnect()
                    
    def connect(self, port=None, right_axis=0):
        if self.driver:
            self.logger.info("Already connected. Disconnecting and reconnecting.")
        
        try:
            self.driver = odrive.find_any(timeout=30, logger=self.logger)
            self.axes = (self.driver.axis0, self.driver.axis1)
        except:
            self.logger.error("No ODrive found. Is device powered?")
            return False
            
        # save some parameters for easy access
        self.right_axis = self.driver.axis0 if right_axis == 0 else self.driver.axis1
        self.left_axis  = self.driver.axis1 if right_axis == 0 else self.driver.axis0
        self.encoder_cpr = self.driver.axis0.encoder.config.cpr
        
        self.connected = True
        return True
        
    def disconnect(self):
        self.connected = False
        self.right_axis = None
        self.left_axis = None
        
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        
        temp_driver = self.driver
        self.driver = None
        try:
            temp_driver.release()
        except:
            return False
        return True

    def calibrate(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        
        self.logger.info("Vbus %.2fV" % self.driver.vbus_voltage)
        
        for i, axis in enumerate(self.axes):
            self.logger.info("Calibrating axis %d..." % i)
            axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            time.sleep(1)
            while axis.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            if axis.error != 0:
                self.logger.error("Failed calibration with axis error 0x%x, motor error 0x%x" % (axis.error, axis.motor.error))
                return False
                
        return True
        
    def preroll(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
            
        self.logger.info("Vbus %.2fV" % self.driver.vbus_voltage)

        for i, axis in enumerate(self.axes):
            self.logger.info("Index search preroll axis %d..." % i)
            axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        
        for i, axis in enumerate(self.axes):
            while axis.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)

        for i, axis in enumerate(self.axes):
            if axis.error != 0:
                self.logger.error("Failed preroll with axis error 0x%x, motor error 0x%x" % (axis.error, axis.motor.error))
                return False
                
        return True
        
        
    def engage(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False

        self.logger.debug("Setting drive mode.")
        for axis in self.axes:
            axis.controller.vel_setpoint = 0
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        
        return True
        
    def release(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return
        self.logger.debug("Releasing.")
        for axis in self.axes: 
            axis.requested_state = AXIS_STATE_IDLE
    
    def drive(self, left_motor_val, right_motor_val):
        if not self.driver:
            self.logger.error("Not connected.")
            return
            
        self.left_axis.controller.vel_setpoint = left_motor_val
        self.right_axis.controller.vel_setpoint = -right_motor_val

