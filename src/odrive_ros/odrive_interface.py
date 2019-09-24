import serial
from serial.serialutil import SerialException

import sys
import time
import logging
import traceback

import odrive
from odrive.enums import *

import fibre
from fibre import ChannelBrokenException, ChannelDamagedException

default_logger = logging.getLogger(__name__)
default_logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

default_logger.addHandler(ch)

class ODriveFailure(Exception):
    pass

class ODriveInterfaceAPI(object):
    driver = None
    encoder_cpr = 4096
    right_axis = None
    left_axis = None
    connected = False
    _preroll_started = False
    _preroll_completed = False
    #engaged = False
    
    def __init__(self, logger=None, active_odrive=None):
        self.logger = logger if logger else default_logger
        
        if active_odrive: # pass in the odrv0 object from odrivetool shell to use it directly.
            self.driver = active_odrive
            self.axes = (self.driver.axis0, self.driver.axis1)
            self.right_axis = self.driver.axis0 
            self.left_axis  = self.driver.axis1
            self.logger.info("Loaded pre-existing ODrive interface. Check index search status.")
            self.encoder_cpr = self.driver.axis0.encoder.config.cpr
            self.connected = True
            self._preroll_started = False
            self._preroll_completed = True
                
    def __del__(self):
        self.disconnect()
        
    def update_time(self, curr_time):
        # provided so simulator can update position
        pass
                    
    def connect(self, port=None, right_axis=0, timeout=30):
        if self.driver:
            self.logger.info("Already connected. Disconnecting and reconnecting.")
        try:
            self.driver = odrive.find_any(timeout=timeout, logger=self.logger)
            self.axes = (self.driver.axis0, self.driver.axis1)
        except:
            self.logger.error("No ODrive found. Is device powered?")
            return False
                        
        # save some parameters for easy access
        self.right_axis = self.driver.axis0 if right_axis == 0 else self.driver.axis1
        self.left_axis  = self.driver.axis1 if right_axis == 0 else self.driver.axis0
        
        # check for no errors
        for axis in [self.right_axis, self.left_axis]:
            if axis.error != 0:
                error_str = "Had error on startup, rebooting. Axis error 0x%x, motor error 0x%x, encoder error 0x%x. Rebooting." % (axis.error, axis.motor.error, axis.encoder.error)
                self.logger.error(error_str)
                self.reboot()
                return False
        
        self.encoder_cpr = self.driver.axis0.encoder.config.cpr
        
        self.connected = True
        self.logger.info("Connected to ODrive. " + self.get_version_string())
        
        self._preroll_started = False
        self._preroll_completed = False
        
        return True
        
    def disconnect(self):
        self.connected = False
        self.right_axis = None
        self.left_axis = None
        
        #self.engaged = False
        
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        
        try:
            self.release()
        except:
            self.logger.error("Error in timer: " + traceback.format_exc())
            return False
        finally:
            self.driver = None
        return True
        
    def get_version_string(self):
        if not self.driver or not self.connected:
            return "Not connected."
        return "ODrive %s, hw v%d.%d-%d, fw v%d.%d.%d%s, sdk v%s" % (
            str(self.driver.serial_number),
            self.driver.hw_version_major, self.driver.hw_version_minor, self.driver.hw_version_variant,
            self.driver.fw_version_major, self.driver.fw_version_minor, self.driver.fw_version_revision,
            "-dev" if self.driver.fw_version_unreleased else "",
            odrive.version.get_version_str())
        
        
    def reboot(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        try:
            self.driver.reboot()
        except KeyError:
            self.logger.error("Rebooted ODrive.")
        except:
            self.logger.error("Failed to reboot: " + traceback.format_exc())
        finally:
            self.driver = None
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
        
    def preroll(self, wait=True):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
            
        if self._preroll_started: # must be prerolling or already prerolled
            return False
        self._preroll_started = True
        self._preroll_completed = False
            
        #self.logger.info("Vbus %.2fV" % self.driver.vbus_voltage)
        
        for i, axis in enumerate(self.axes):
            self.logger.info("Index search preroll axis %d..." % i)
            axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        
        if wait:
            for i, axis in enumerate(self.axes):
                while axis.current_state != AXIS_STATE_IDLE:
                    time.sleep(0.1)
            self._preroll_started = False
            for i, axis in enumerate(self.axes):
                if axis.error != 0:
                    self.logger.error("Failed preroll with left_axis error 0x%x, motor error 0x%x" % (axis.error, axis.motor.error))
                    return False
            self._preroll_completed = True
            self.logger.info("Index search preroll complete.")
            return True
        else:
            return False
        
    # def prerolling(self):
    #     return self.axes[0].current_state == AXIS_STATE_ENCODER_INDEX_SEARCH or self.axes[1].current_state == AXIS_STATE_ENCODER_INDEX_SEARCH
    #
    # def prerolled(self): #
    #     return self._prerolled and not self.prerolling()
        
    def ensure_prerolled(self):
        # preroll success
        if self._preroll_completed:
            return True
        # started, not completed
        elif self._preroll_started:
            #self.logger.info("Checking for preroll complete.")
            if self.axes[0].current_state != AXIS_STATE_ENCODER_INDEX_SEARCH and self.axes[1].current_state != AXIS_STATE_ENCODER_INDEX_SEARCH:
                # completed, check for errors before marking complete
                for i, axis in enumerate(self.axes):
                    if axis.error != 0:
                        self._preroll_started = False
                        error_str = "Failed index search preroll with axis error 0x%x, motor error 0x%x, encoder error 0x%x. Rebooting." % (axis.error, axis.motor.error, axis.encoder.error)
                        #self.reboot()
                        self.logger.error(error_str)
                        raise Exception(error_str)
                # no errors, success
                self._preroll_started = False
                self._preroll_completed = True
                self.logger.info("Index search preroll complete. Ready to drive.")
                return True
            else:
                # still prerolling
                return False
        else: # start preroll
            #self.logger.info("Preroll started.")
            self.preroll(wait=False)
            return False
            
    def has_prerolled(self):
        return self._preroll_completed
    
    def engaged(self):
        if self.driver and hasattr(self, 'axes'):
            return self.axes[0].current_state == AXIS_STATE_CLOSED_LOOP_CONTROL and self.axes[1].current_state == AXIS_STATE_CLOSED_LOOP_CONTROL
        else:
            return False
    
    def idle(self):
        if self.driver and hasattr(self, 'axes'):
            return self.axes[0].current_state == AXIS_STATE_IDLE and self.axes[1].current_state == AXIS_STATE_IDLE
        else:
            return False
        
    def engage(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False

        #self.logger.debug("Setting drive mode.")
        for axis in self.axes:
            axis.controller.vel_setpoint = 0
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        
        #self.engaged = True
        return True
        
    def release(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        #self.logger.debug("Releasing.")
        for axis in self.axes: 
            axis.requested_state = AXIS_STATE_IDLE

        #self.engaged = False
        return True
    
    def drive(self, left_motor_val, right_motor_val):
        if not self.driver:
            self.logger.error("Not connected.")
            return
        #try:
        self.left_axis.controller.vel_setpoint = left_motor_val
        self.right_axis.controller.vel_setpoint = -right_motor_val
        #except (fibre.protocol.ChannelBrokenException, AttributeError) as e:
        #    raise ODriveFailure(str(e))
        
    def feed_watchdog(self):
        self.left_axis.watchdog_feed()
        self.right_axis.watchdog_feed()
        
    def get_errors(self, clear=True):
        # TODO: add error parsing, see: https://github.com/madcowswe/ODrive/blob/master/tools/odrive/utils.py#L34
        if not self.driver:
            return None
            
        axis_error = self.axes[0].error or self.axes[1].error
        
        if axis_error:
            error_string = "Errors(hex): L: a%x m%x e%x c%x, R: a%x m%x e%x c%x" % (
                self.left_axis.error,  self.left_axis.motor.error,  self.left_axis.encoder.error,  self.left_axis.controller.error,
                self.right_axis.error, self.right_axis.motor.error, self.right_axis.encoder.error, self.right_axis.controller.error,
            )
        
        if clear:
            for axis in self.axes:
                axis.error = 0
                axis.motor.error = 0
                axis.encoder.error = 0
                axis.controller.error = 0
        
        if axis_error:
            return error_string
            
    def left_vel_estimate(self):  return self.left_axis.encoder.vel_estimate   if self.left_axis  else 0 # units: encoder counts/s
    def right_vel_estimate(self): return self.right_axis.encoder.vel_estimate  if self.right_axis else 0 # neg is forward for right
    def left_pos(self):           return self.left_axis.encoder.pos_cpr        if self.left_axis  else 0  # units: encoder counts
    def right_pos(self):          return self.right_axis.encoder.pos_cpr       if self.right_axis else 0   # sign!
    
    # TODO check these match the right motors, but it doesn't matter for now
    def left_temperature(self):   return self.left_axis.motor.get_inverter_temp()  if self.left_axis  else 0.
    def right_temperature(self):  return self.right_axis.motor.get_inverter_temp() if self.right_axis else 0.
    
    def left_current(self):       return self.left_axis.motor.current_control.Ibus  if self.left_axis and self.left_axis.current_state > 1 else 0.
    def right_current(self):      return self.right_axis.motor.current_control.Ibus if self.right_axis and self.right_axis.current_state > 1 else 0.
    
    # from axis.hpp: https://github.com/madcowswe/ODrive/blob/767a2762f9b294b687d761029ef39e742bdf4539/Firmware/MotorControl/axis.hpp#L26
    MOTOR_STATES = [
        "UNDEFINED",                  #<! will fall through to idle
        "IDLE",                       #<! disable PWM and do nothing
        "STARTUP_SEQUENCE",           #<! the actual sequence is defined by the config.startup_... flags
        "FULL_CALIBRATION_SEQUENCE",  #<! run all calibration procedures, then idle
        "MOTOR_CALIBRATION",          #//<! run motor calibration
        "SENSORLESS_CONTROL",         #//<! run sensorless control
        "ENCODER_INDEX_SEARCH",       #//<! run encoder index search
        "ENCODER_OFFSET_CALIBRATION", #//<! run encoder offset calibration
        "CLOSED_LOOP_CONTROL",        #//<! run closed loop control
        "LOCKIN_SPIN",                #//<! run lockin spin
        "ENCODER_DIR_FIND",
        ]
        
    def left_state(self):       return self.MOTOR_STATES[self.left_axis.current_state] if self.left_axis else "NOT_CONNECTED"
    def right_state(self):      return self.MOTOR_STATES[self.right_axis.current_state] if self.right_axis else "NOT_CONNECTED"
    
    def bus_voltage(self):      return self.driver.vbus_voltage if self.left_axis else 0.
    
