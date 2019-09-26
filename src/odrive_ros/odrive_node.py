#!/usr/bin/env python
from __future__ import print_function

#import roslib; roslib.load_manifest('BINCADDY')
import rospy
#import roslib
import tf.transformations
import tf_conversions
import tf2_ros

import std_msgs.msg
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import std_srvs.srv

#roslib.load_manifest('diagnostic_updater')
import diagnostic_updater, diagnostic_msgs.msg

import time
import math
import traceback
import Queue

from odrive_interface import ODriveInterfaceAPI, ODriveFailure
from odrive_interface import ChannelBrokenException, ChannelDamagedException
from odrive_simulator import ODriveInterfaceSimulator

class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def debug(self, msg):    rospy.logdebug(msg)  #  print(msg) #
    def info(self, msg):     rospy.loginfo(msg)   #  print(msg) #
    def warn(self, msg):     rospy.logwarn(msg)   #  print(msg) #
    def error(self, msg):    rospy.logerr(msg)    #  print(msg) #
    def critical(self, msg): rospy.logfatal(msg)  #  print(msg) #
    
    # use_index = False (bool)
    # offset_float = 0.590887010098 (float)
    # calib_range = 0.019999999553 (float)
    # mode = 0 (int)
    # offset = 1809 (int)
    # cpr = 4096 (int)
    # idx_search_speed = 10.0 (float)
    # pre_calibrated = False (bool)

#m_s_to_rpm = 60.0/tyre_circumference
#m_s_to_erpm = 10 * m_s_to_rpm 

# 4096 counts / rev, so 4096 == 1 rev/s


# 1 m/s = 3.6 km/hr

def get_param(name, default):
    val = rospy.get_param(name, default)
    rospy.loginfo('  %s: %s', name, str(val))
    return val

class ODriveNode(object):
    last_speed = 0.0
    driver = None
    prerolling = False
    
    # Robot wheel_track params for velocity -> motor speed conversion
    wheel_track = None
    tyre_circumference = None
    encoder_counts_per_rev = None
    m_s_to_value = 1.0
    axis_for_right = 0
    encoder_cpr = 4096
    
    # Startup parameters
    connect_on_startup = False
    calibrate_on_startup = False
    engage_on_startup = False
    
    publish_joint_angles = True
    # Simulation mode
    # When enabled, output simulated odometry and joint angles (TODO: do joint angles anyway from ?)
    sim_mode = False
    
    def __init__(self):
        self.sim_mode             = get_param('simulation_mode', False)
        self.publish_joint_angles = get_param('publish_joint_angles', True) # if self.sim_mode else False
        self.publish_temperatures = get_param('publish_temperatures', True)
        
        self.axis_for_right = float(get_param('~axis_for_right', 0)) # if right calibrates first, this should be 0, else 1
        self.wheel_track = float(get_param('~wheel_track', 0.285)) # m, distance between wheel centres
        self.tyre_circumference = float(get_param('~tyre_circumference', 0.341)) # used to translate velocity commands in m/s into motor rpm
        
        self.connect_on_startup   = get_param('~connect_on_startup', False)
        #self.calibrate_on_startup = get_param('~calibrate_on_startup', False)
        #self.engage_on_startup    = get_param('~engage_on_startup', False)
        
        self.has_preroll     = get_param('~use_preroll', True)
                
        self.publish_current = get_param('~publish_current', True)
        self.publish_raw_odom =get_param('~publish_raw_odom', True)
        
        self.publish_odom    = get_param('~publish_odom', True)
        self.publish_tf      = get_param('~publish_odom_tf', False)
        self.odom_topic      = get_param('~odom_topic', "odom")
        self.odom_frame      = get_param('~odom_frame', "odom")
        self.base_frame      = get_param('~base_frame', "base_link")
        self.odom_calc_hz    = get_param('~odom_calc_hz', 10)
        
        rospy.on_shutdown(self.terminate)

        rospy.Service('connect_driver',    std_srvs.srv.Trigger, self.connect_driver)
        rospy.Service('disconnect_driver', std_srvs.srv.Trigger, self.disconnect_driver)
    
        rospy.Service('calibrate_motors',         std_srvs.srv.Trigger, self.calibrate_motor)
        rospy.Service('engage_motors',            std_srvs.srv.Trigger, self.engage_motor)
        rospy.Service('release_motors',           std_srvs.srv.Trigger, self.release_motor)
                
        # odometry update, disable during preroll, whenever wheels off ground 
        self.odometry_update_enabled = True
        rospy.Service('enable_odometry_updates', std_srvs.srv.SetBool, self.enable_odometry_update_service)
        
        self.status_pub = rospy.Publisher('status', std_msgs.msg.String, latch=True, queue_size=2)
        self.status = "disconnected"
        self.status_pub.publish(self.status)
        
        self.command_queue = Queue.Queue(maxsize=5)
        self.vel_subscribe = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=2)
        
        self.publish_diagnostics = True
        if self.publish_diagnostics:
            self.diagnostic_updater = diagnostic_updater.Updater()
            self.diagnostic_updater.setHardwareID("Not connected, unknown")
            self.diagnostic_updater.add("ODrive Diagnostics", self.pub_diagnostics)
        
        if self.publish_temperatures:
            self.temperature_publisher_left  = rospy.Publisher('left/temperature', Float64, queue_size=2)
            self.temperature_publisher_right = rospy.Publisher('right/temperature', Float64, queue_size=2)
        
        self.i2t_error_latch = False
        if self.publish_current:
            #self.current_loop_count = 0
            #self.left_current_accumulator  = 0.0
            #self.right_current_accumulator = 0.0
            self.current_publisher_left  = rospy.Publisher('left/current', Float64, queue_size=2)
            self.current_publisher_right = rospy.Publisher('right/current', Float64, queue_size=2)
            self.i2t_publisher_left  = rospy.Publisher('left/i2t', Float64, queue_size=2)
            self.i2t_publisher_right = rospy.Publisher('right/i2t', Float64, queue_size=2)
            
            rospy.logdebug("ODrive will publish motor currents.")
            
            self.i2t_resume_threshold  = get_param('~i2t_resume_threshold',  222)            
            self.i2t_warning_threshold = get_param('~i2t_warning_threshold', 333)
            self.i2t_error_threshold   = get_param('~i2t_error_threshold',   666)
        
        self.last_cmd_vel_time = rospy.Time.now()
        
        if self.publish_raw_odom:
            self.raw_odom_publisher_encoder_left  = rospy.Publisher('left/raw_odom/encoder',   Int32, queue_size=2) if self.publish_raw_odom else None
            self.raw_odom_publisher_encoder_right = rospy.Publisher('right/raw_odom/encoder',  Int32, queue_size=2) if self.publish_raw_odom else None
            self.raw_odom_publisher_vel_left      = rospy.Publisher('left/raw_odom/velocity',  Int32, queue_size=2) if self.publish_raw_odom else None
            self.raw_odom_publisher_vel_right     = rospy.Publisher('right/raw_odom/velocity', Int32, queue_size=2) if self.publish_raw_odom else None
                            
        if self.publish_odom:
            rospy.Service('reset_odometry',    std_srvs.srv.Trigger, self.reset_odometry)
            self.old_pos_l = 0
            self.old_pos_r = 0
            
            self.odom_publisher  = rospy.Publisher(self.odom_topic, Odometry, tcp_nodelay=True, queue_size=2)
            # setup message
            self.odom_msg = Odometry()
            #print(dir(self.odom_msg))
            self.odom_msg.header.frame_id = self.odom_frame
            self.odom_msg.child_frame_id = self.base_frame
            self.odom_msg.pose.pose.position.x = 0.0
            self.odom_msg.pose.pose.position.y = 0.0
            self.odom_msg.pose.pose.position.z = 0.0    # always on the ground, we hope
            self.odom_msg.pose.pose.orientation.x = 0.0 # always vertical
            self.odom_msg.pose.pose.orientation.y = 0.0 # always vertical
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
            self.odom_msg.twist.twist.linear.x = 0.0
            self.odom_msg.twist.twist.linear.y = 0.0  # no sideways
            self.odom_msg.twist.twist.linear.z = 0.0  # or upwards... only forward
            self.odom_msg.twist.twist.angular.x = 0.0 # or roll
            self.odom_msg.twist.twist.angular.y = 0.0 # or pitch... only yaw
            self.odom_msg.twist.twist.angular.z = 0.0
            
            # store current location to be updated. 
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            
            # setup transform
            self.tf_publisher = tf2_ros.TransformBroadcaster()
            self.tf_msg = TransformStamped()
            self.tf_msg.header.frame_id = self.odom_frame
            self.tf_msg.child_frame_id  = self.base_frame
            self.tf_msg.transform.translation.x = 0.0
            self.tf_msg.transform.translation.y = 0.0
            self.tf_msg.transform.translation.z = 0.0
            self.tf_msg.transform.rotation.x = 0.0
            self.tf_msg.transform.rotation.y = 0.0
            self.tf_msg.transform.rotation.w = 0.0
            self.tf_msg.transform.rotation.z = 1.0
            
        if self.publish_joint_angles:
            self.joint_state_publisher = rospy.Publisher('/odrive/joint_states', JointState, queue_size=2)
            
            jsm = JointState()
            self.joint_state_msg = jsm
            #jsm.name.resize(2)
            #jsm.position.resize(2)
            jsm.name = ['joint_left_wheel','joint_right_wheel']
            jsm.position = [0.0, 0.0]            

        
    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        main_rate = rospy.Rate(1) # hz
        # Start timer to run high-rate comms
        self.fast_timer = rospy.Timer(rospy.Duration(1/float(self.odom_calc_hz)), self.fast_timer)
        
        self.fast_timer_comms_active = False
        
        while not rospy.is_shutdown():
            try:
                main_rate.sleep()
            except rospy.ROSInterruptException: # shutdown / stop ODrive??
                break;
            
            # fast timer running, so do nothing and wait for any errors
            if self.fast_timer_comms_active:
                continue
            
            # check for errors
            if self.driver:
                try:
                    # driver connected, but fast_comms not active -> must be an error?
                    # TODO: try resetting errors and recalibrating, not just a full disconnection
                    error_string = self.driver.get_errors(clear=True)
                    if error_string:
                        rospy.logerr("Had errors, disconnecting and retrying connection.")
                        rospy.logerr(error_string)
                        self.driver.disconnect()
                        self.status = "disconnected"
                        self.status_pub.publish(self.status)
                        self.driver = None
                    else:
                        # must have called connect service from another node
                        self.fast_timer_comms_active = True
                except (ChannelBrokenException, ChannelDamagedException, AttributeError):
                    rospy.logerr("ODrive USB connection failure in main_loop.")
                    self.status = "disconnected"
                    self.status_pub.publish(self.status)
                    self.driver = None
                except:
                    rospy.logerr("Unknown errors accessing ODrive:" + traceback.format_exc())
                    self.status = "disconnected"
                    self.status_pub.publish(self.status)
                    self.driver = None
            
            if not self.driver:
                if not self.connect_on_startup:
                    #rospy.loginfo("ODrive node started, but not connected.")
                    continue
                
                if not self.connect_driver(None)[0]:
                    rospy.logerr("Failed to connect.") # TODO: can we check for timeout here?
                    continue
                    
                if self.publish_diagnostics:
                    self.diagnostic_updater.setHardwareID(self.driver.get_version_string())
            
            else:
                pass # loop around and try again
        
    def fast_timer(self, timer_event):
        time_now = rospy.Time.now()
        # in case of failure, assume some values are zero
        self.vel_l = 0
        self.vel_r = 0
        self.new_pos_l = 0
        self.new_pos_r = 0
        self.current_l = 0
        self.current_r = 0
        self.temp_v_l = 0.
        self.temp_v_r = 0.
        self.motor_state_l = "not connected" # undefined
        self.motor_state_r = "not connected"
        self.bus_voltage = 0.
        
        # Handle reading from Odrive and sending odometry
        if self.fast_timer_comms_active:
            try:
                # check errors
                error_string = self.driver.get_errors()
                if error_string:
                    self.fast_timer_comms_active = False
                else:
                    # reset watchdog
                    self.driver.feed_watchdog()
                    
                    # read all required values from ODrive for odometry
                    self.motor_state_l = self.driver.left_state()
                    self.motor_state_r = self.driver.right_state()
                    
                    self.encoder_cpr = self.driver.encoder_cpr
                    self.m_s_to_value = self.encoder_cpr/self.tyre_circumference # calculated
                
                    self.driver.update_time(time_now.to_sec())
                    self.vel_l = self.driver.left_vel_estimate()   # units: encoder counts/s
                    self.vel_r = -self.driver.right_vel_estimate() # neg is forward for right
                    self.new_pos_l = self.driver.left_pos()        # units: encoder counts
                    self.new_pos_r = -self.driver.right_pos()      # sign!
                    
                    # for temperatures
                    self.temp_v_l = self.driver.left_temperature()
                    self.temp_v_r = self.driver.right_temperature()
                    # for current
                    self.current_l = self.driver.left_current()
                    self.current_r = self.driver.right_current()
                    # voltage
                    self.bus_voltage = self.driver.bus_voltage()
                    
            except (ChannelBrokenException, ChannelDamagedException):
                rospy.logerr("ODrive USB connection failure in fast_timer." + traceback.format_exc(1))
                self.fast_timer_comms_active = False
                self.status = "disconnected"
                self.status_pub.publish(self.status)
                self.driver = None
            except:
                rospy.logerr("Fast timer ODrive failure:" + traceback.format_exc())
                self.fast_timer_comms_active = False
                
        # odometry is published regardless of ODrive connection or failure (but assumed zero for those)
        # as required by SLAM
        if self.publish_odom:
            self.pub_odometry(time_now)
        if self.publish_temperatures:
            self.pub_temperatures()
        if self.publish_current:
            self.pub_current()
        if self.publish_joint_angles:
            self.pub_joint_angles(time_now)
        if self.publish_diagnostics:
            self.diagnostic_updater.update()
        
        try:
            # check and stop motor if no vel command has been received in > 1s
            #if self.fast_timer_comms_active:
            if self.driver:
                if (time_now - self.last_cmd_vel_time).to_sec() > 0.5 and self.last_speed > 0:
                    self.driver.drive(0,0)
                    self.last_speed = 0
                    self.last_cmd_vel_time = time_now
                # release motor after 10s stopped
                if (time_now - self.last_cmd_vel_time).to_sec() > 10.0 and self.driver.engaged():
                    self.driver.release() # and release            
        except (ChannelBrokenException, ChannelDamagedException):
            rospy.logerr("ODrive USB connection failure in cmd_vel timeout." + traceback.format_exc(1))
            self.fast_timer_comms_active = False
            self.driver = None
        except:
            rospy.logerr("cmd_vel timeout unknown failure:" + traceback.format_exc())
            self.fast_timer_comms_active = False

        
        # handle sending drive commands.
        # from here, any errors return to get out
        if self.fast_timer_comms_active and not self.command_queue.empty():
            # check to see if we're initialised and engaged motor
            try:
                if not self.driver.has_prerolled(): #ensure_prerolled():
                    rospy.logwarn_throttle(5.0, "ODrive has not been prerolled, ignoring drive command.")
                    motor_command = self.command_queue.get_nowait()
                    return
            except:
                rospy.logerr("Fast timer exception on preroll." + traceback.format_exc())
                self.fast_timer_comms_active = False                
            try:
                motor_command = self.command_queue.get_nowait()
            except Queue.Empty:
                rospy.logerr("Queue was empty??" + traceback.format_exc())
                return
            
            if motor_command[0] == 'drive':
                try:
                    if self.publish_current and self.i2t_error_latch:
                        # have exceeded i2t bounds
                        return
                    
                    if not self.driver.engaged():
                        self.driver.engage()
                        self.status = "engaged"
                        
                    left_linear_val, right_linear_val = motor_command[1]
                    self.driver.drive(left_linear_val, right_linear_val)
                    self.last_speed = max(abs(left_linear_val), abs(right_linear_val))
                    self.last_cmd_vel_time = time_now
                except (ChannelBrokenException, ChannelDamagedException):
                    rospy.logerr("ODrive USB connection failure in drive_cmd." + traceback.format_exc(1))
                    self.fast_timer_comms_active = False
                    self.driver = None
                except:
                    rospy.logerr("motor drive unknown failure:" + traceback.format_exc())
                    self.fast_timer_comms_active = False

            elif motor_command[0] == 'release':
                pass
            # ?
            else:
                pass
    
    
    def terminate(self):
        self.fast_timer.shutdown()
        if self.driver:
            self.driver.release()
    
    # ROS services
    def connect_driver(self, request):
        if self.driver:
            return (False, "Already connected.")
        
        ODriveClass = ODriveInterfaceAPI if not self.sim_mode else ODriveInterfaceSimulator
        
        self.driver = ODriveInterfaceAPI(logger=ROSLogger())
        rospy.loginfo("Connecting to ODrive...")
        if not self.driver.connect(right_axis=self.axis_for_right):
            self.driver = None
            #rospy.logerr("Failed to connect.")
            return (False, "Failed to connect.")
            
        #rospy.loginfo("ODrive connected.")
        
        # okay, connected, 
        self.m_s_to_value = self.driver.encoder_cpr/self.tyre_circumference
        
        if self.publish_odom:
            self.old_pos_l = self.driver.left_axis.encoder.pos_cpr
            self.old_pos_r = self.driver.right_axis.encoder.pos_cpr
        
        self.fast_timer_comms_active = True
        
        self.status = "connected"
        self.status_pub.publish(self.status)
        return (True, "ODrive connected successfully")
    
    def disconnect_driver(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        try:
            if not self.driver.disconnect():
                return (False, "Failed disconnection, but try reconnecting.")
        except:
            rospy.logerr('Error while disconnecting: {}'.format(traceback.format_exc()))
        finally:
            self.status = "disconnected"
            self.status_pub.publish(self.status_pub)
            self.driver = None
        return (True, "Disconnection success.")
    
    def calibrate_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
            
        if self.has_preroll:
            self.odometry_update_enabled = False # disable odometry updates while we preroll
            if not self.driver.preroll(wait=True):
                self.status = "preroll_fail"
                self.status_pub.publish(self.status)
                return (False, "Failed preroll.")
            
            self.status_pub.publish("ready")
            rospy.sleep(1)
            self.odometry_update_enabled = True
        else:
            if not self.driver.calibrate():
                return (False, "Failed calibration.")
                
        return (True, "Calibration success.")
    
    def engage_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.has_prerolled():
            return (False, "Not prerolled.")
        if not self.driver.engage():
            return (False, "Failed to engage motor.")
        return (True, "Engage motor success.")
    
    def release_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.release():
            return (False, "Failed to release motor.")
        return (True, "Release motor success.")
        
    def enable_odometry_update_service(self, request):
        enable = request.data
        
        if enable:
            self.odometry_update_enabled = True
            return(True, "Odometry enabled.")
        else:
            self.odometry_update_enabled = False
            return(True, "Odometry disabled.")
    
    def reset_odometry(self, request):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        return(True, "Odometry reset.")
    
    # Helpers and callbacks
    
    def convert(self, forward, ccw):
        angular_to_linear = ccw * (self.wheel_track/2.0) 
        left_linear_val  = int((forward - angular_to_linear) * self.m_s_to_value)
        right_linear_val = int((forward + angular_to_linear) * self.m_s_to_value)
    
        return left_linear_val, right_linear_val

    def cmd_vel_callback(self, msg):
        #rospy.loginfo("Received a /cmd_vel message!")
        #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

        # rostopic pub -r 1 /commands/motor/current std_msgs/Float64 -- -1.0

        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities into motor commands
        
        # 3600 ERPM = 360 RPM ~= 6 km/hr
        
        #angular_to_linear = msg.angular.z * (wheel_track/2.0) 
        #left_linear_rpm  = (msg.linear.x - angular_to_linear) * m_s_to_erpm
        #right_linear_rpm = (msg.linear.x + angular_to_linear) * m_s_to_erpm
        left_linear_val, right_linear_val = self.convert(msg.linear.x, msg.angular.z)
        
        # if wheel speed = 0, stop publishing after sending 0 once. #TODO add error term, work out why VESC turns on for 0 rpm
        
        # Then set your wheel speeds (using wheel_left and wheel_right as examples)
        #self.left_motor_pub.publish(left_linear_rpm)
        #self.right_motor_pub.publish(right_linear_rpm)
        #wheel_left.set_speed(v_l)
        #wheel_right.set_speed(v_r)
        
        #rospy.logdebug("Driving left: %d, right: %d, from linear.x %.2f and angular.z %.2f" % (left_linear_val, right_linear_val, msg.linear.x, msg.angular.z))
        try:
            drive_command = ('drive', (left_linear_val, right_linear_val))
            self.command_queue.put_nowait(drive_command)
        except Queue.Full:
            pass
            
        self.last_cmd_vel_time = rospy.Time.now()
        
    def pub_diagnostics(self, stat):
        stat.add("Status", self.status)
        stat.add("Motor state L", self.motor_state_l) 
        stat.add("Motor state R", self.motor_state_r)
        stat.add("FET temp L (C)", round(self.temp_v_l,1))
        stat.add("FET temp R (C)", round(self.temp_v_r,1))
        stat.add("Motor temp L (C)", "unimplemented")
        stat.add("Motor temp R (C)", "unimplemented")
        stat.add("Motor current L (A)", round(self.current_l,1))
        stat.add("Motor current R (A)", round(self.current_r,1))
        stat.add("Voltage (V)", round(self.bus_voltage,2))
        stat.add("Motor i2t L", round(self.left_energy_acc,1))
        stat.add("Motor i2t R", round(self.right_energy_acc,1))
        
        # https://github.com/ros/common_msgs/blob/jade-devel/diagnostic_msgs/msg/DiagnosticStatus.msg
        if self.status == "disconnected":
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Not connected")
        else:
            if self.i2t_error_latch:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "i2t overheated, drive ignored until cool")
            elif self.left_energy_acc > self.i2t_warning_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Left motor over i2t warning threshold")
            elif self.left_energy_acc > self.i2t_error_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Left motor over i2t error threshold")
            elif self.right_energy_acc > self.i2t_warning_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Right motor over i2t warning threshold")
            elif self.right_energy_acc > self.i2t_error_threshold:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Right motor over i2t error threshold")
            # Everything is okay:
            else:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Running")
        
        
    def pub_temperatures(self):
        # https://discourse.odriverobotics.com/t/odrive-mosfet-temperature-rise-measurements-using-the-onboard-thermistor/972
        # https://discourse.odriverobotics.com/t/thermistors-on-the-odrive/813/7
        # https://www.digikey.com/product-detail/en/murata-electronics-north-america/NCP15XH103F03RC/490-4801-1-ND/1644682
        #p3 =  363.0
        #p2 = -459.2
        #p1 =  308.3
        #p0 =  -28.1
        #
        #vl = self.temp_v_l
        #vr = self.temp_v_r

        #temperature_l = p3*vl**3 + p2*vl**2 + p1*vl + p0
        #temperature_r = p3*vr**3 + p2*vr**2 + p1*vr + p0
        
        #print(temperature_l, temperature_r)
        
        self.temperature_publisher_left.publish(self.temp_v_l)
        self.temperature_publisher_right.publish(self.temp_v_r)
        
    # Current publishing and i2t calculation
    i2t_current_nominal = 2.0
    i2t_update_rate = 0.01
    
    def pub_current(self):
        self.current_publisher_left.publish(float(self.current_l))
        self.current_publisher_right.publish(float(self.current_r))
        
        now = time.time()
        
        if not hasattr(self, 'last_pub_current_time'):
            self.last_pub_current_time = now
            self.left_energy_acc = 0
            self.right_energy_acc = 0
            return
            
        # calculate and publish i2t
        dt = now - self.last_pub_current_time
        
        power = max(0, self.current_l**2 - self.i2t_current_nominal**2)
        energy = power * dt
        self.left_energy_acc *= 1 - self.i2t_update_rate * dt
        self.left_energy_acc += energy
        
        power = max(0, self.current_r**2 - self.i2t_current_nominal**2)
        energy = power * dt
        self.right_energy_acc *= 1 - self.i2t_update_rate * dt
        self.right_energy_acc += energy
        
        self.last_pub_current_time = now
        
        self.i2t_publisher_left.publish(float(self.left_energy_acc))
        self.i2t_publisher_right.publish(float(self.right_energy_acc))
        
        # stop odrive if overheated
        if self.left_energy_acc > self.i2t_error_threshold or self.right_energy_acc > self.i2t_error_threshold:
            if not self.i2t_error_latch:
                self.driver.release()
                self.status = "overheated"
                self.i2t_error_latch = True
                rospy.logerr("ODrive has exceeded i2t error threshold, ignoring drive commands. Waiting to cool down.")
        elif self.i2t_error_latch:
            if self.left_energy_acc < self.i2t_resume_threshold and self.right_energy_acc < self.i2t_resume_threshold:
                # have cooled enough now
                self.status = "ready"
                self.i2t_error_latch = False
                rospy.logerr("ODrive has cooled below i2t resume threshold, ignoring drive commands. Waiting to cool down.")
        
        
    #     current_quantizer = 5
    #
    #     self.left_current_accumulator += self.current_l
    #     self.right_current_accumulator += self.current_r
    #
    #     self.current_loop_count += 1
    #     if self.current_loop_count >= current_quantizer:
    #         self.current_publisher_left.publish(float(self.left_current_accumulator) / current_quantizer)
    #         self.current_publisher_right.publish(float(self.right_current_accumulator) / current_quantizer)
    #
    #         self.current_loop_count = 0
    #         self.left_current_accumulator = 0.0
    #         self.right_current_accumulator = 0.0

    def pub_odometry(self, time_now):
        now = time_now
        self.odom_msg.header.stamp = now
        self.tf_msg.header.stamp = now
        
        wheel_track = self.wheel_track   # check these. Values in m
        tyre_circumference = self.tyre_circumference
        # self.m_s_to_value = encoder_cpr/tyre_circumference set earlier
        
        # if odometry updates disabled, just return the old position and zero twist.
        if not self.odometry_update_enabled:
            self.odom_msg.twist.twist.linear.x = 0.
            self.odom_msg.twist.twist.angular.z = 0.
            
            # but update the old encoder positions, so when we restart updates
            # it will start by giving zero change from the old position.
            self.old_pos_l = self.new_pos_l
            self.old_pos_r = self.new_pos_r
            
            self.odom_publisher.publish(self.odom_msg)
            if self.publish_tf:
                self.tf_publisher.sendTransform(self.tf_msg)
            
            return
        
        # Twist/velocity: calculated from motor values only
        s = tyre_circumference * (self.vel_l+self.vel_r) / (2.0*self.encoder_cpr)
        w = tyre_circumference * (self.vel_r-self.vel_l) / (wheel_track * self.encoder_cpr) # angle: vel_r*tyre_radius - vel_l*tyre_radius
        self.odom_msg.twist.twist.linear.x = s
        self.odom_msg.twist.twist.angular.z = w
    
        #rospy.loginfo("vel_l: % 2.2f  vel_r: % 2.2f  vel_l: % 2.2f  vel_r: % 2.2f  x: % 2.2f  th: % 2.2f  pos_l: % 5.1f pos_r: % 5.1f " % (
        #                vel_l, -vel_r,
        #                vel_l/encoder_cpr, vel_r/encoder_cpr, self.odom_msg.twist.twist.linear.x, self.odom_msg.twist.twist.angular.z,
        #                self.driver.left_axis.encoder.pos_cpr, self.driver.right_axis.encoder.pos_cpr))
        
        # Position
        delta_pos_l = self.new_pos_l - self.old_pos_l
        delta_pos_r = self.new_pos_r - self.old_pos_r
        
        self.old_pos_l = self.new_pos_l
        self.old_pos_r = self.new_pos_r
        
        # Check for overflow. Assume we can't move more than half a circumference in a single timestep. 
        half_cpr = self.encoder_cpr/2.0
        if   delta_pos_l >  half_cpr: delta_pos_l = delta_pos_l - self.encoder_cpr
        elif delta_pos_l < -half_cpr: delta_pos_l = delta_pos_l + self.encoder_cpr
        if   delta_pos_r >  half_cpr: delta_pos_r = delta_pos_r - self.encoder_cpr
        elif delta_pos_r < -half_cpr: delta_pos_r = delta_pos_r + self.encoder_cpr
        
        # counts to metres
        delta_pos_l_m = delta_pos_l / self.m_s_to_value
        delta_pos_r_m = delta_pos_r / self.m_s_to_value
    
        # Distance travelled
        d = (delta_pos_l_m+delta_pos_r_m)/2.0  # delta_ps
        th = (delta_pos_r_m-delta_pos_l_m)/wheel_track # works for small angles
    
        xd = math.cos(th)*d
        yd = -math.sin(th)*d
    
        # elapsed time = event.last_real, event.current_real
        #elapsed = (event.current_real-event.last_real).to_sec()
        # calc_vel: d/elapsed, th/elapsed
    
        # Pose: updated from previous pose + position delta
        self.x += math.cos(self.theta)*xd - math.sin(self.theta)*yd
        self.y += math.sin(self.theta)*xd + math.cos(self.theta)*yd
        self.theta = (self.theta + th) % (2*math.pi)
    
        #rospy.loginfo("dl_m: % 2.2f  dr_m: % 2.2f  d: % 2.2f  th: % 2.2f  xd: % 2.2f  yd: % 2.2f  x: % 5.1f y: % 5.1f  th: % 5.1f" % (
        #                delta_pos_l_m, delta_pos_r_m,
        #                d, th, xd, yd,
        #                self.x, self.y, self.theta
        #                ))
    
        # fill odom message and publish
        
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        self.odom_msg.pose.pose.orientation.z = q[2] # math.sin(self.theta)/2
        self.odom_msg.pose.pose.orientation.w = q[3] # math.cos(self.theta)/2
    
        #rospy.loginfo("theta: % 2.2f  z_m: % 2.2f  w_m: % 2.2f  q[2]: % 2.2f  q[3]: % 2.2f (q[0]: %2.2f  q[1]: %2.2f)" % (
        #                        self.theta,
        #                        math.sin(self.theta)/2, math.cos(self.theta)/2,
        #                        q[2],q[3],q[0],q[1]
        #                        ))
    
        #self.odom_msg.pose.covariance
         # x y z
         # x y z
    
        self.tf_msg.transform.translation.x = self.x
        self.tf_msg.transform.translation.y = self.y
        #self.tf_msg.transform.rotation.x
        #self.tf_msg.transform.rotation.x
        self.tf_msg.transform.rotation.z = q[2]
        self.tf_msg.transform.rotation.w = q[3]
        
        if self.publish_raw_odom:
            self.raw_odom_publisher_encoder_left.publish(self.new_pos_l)
            self.raw_odom_publisher_encoder_right.publish(self.new_pos_r)
            self.raw_odom_publisher_vel_left.publish(self.vel_l)
            self.raw_odom_publisher_vel_right.publish(self.vel_r)
        
        # ... and publish!
        self.odom_publisher.publish(self.odom_msg)
        if self.publish_tf:
            self.tf_publisher.sendTransform(self.tf_msg)            
    
    def pub_joint_angles(self, time_now):
        jsm = self.joint_state_msg
        jsm.header.stamp = time_now
        if self.driver:
            jsm.position[0] = 2*math.pi * self.new_pos_l  / self.encoder_cpr
            jsm.position[1] = 2*math.pi * self.new_pos_r / self.encoder_cpr
            
        self.joint_state_publisher.publish(jsm)

def start_odrive():
    rospy.init_node('odrive')
    odrive_node = ODriveNode()
    odrive_node.main_loop()
    #rospy.spin() 
    
if __name__ == '__main__':
    try:
        start_odrive()
    except rospy.ROSInterruptException:
        pass

