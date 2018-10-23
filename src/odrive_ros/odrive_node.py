#!/usr/bin/env python
from __future__ import print_function

#import roslib; roslib.load_manifest('BINCADDY')
import rospy
import tf.transformations

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import std_srvs.srv

import time

from odrive_interface import ODriveInterfaceSerial, ODriveInterfaceAPI

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

class ODriveNode(object):
    last_speed = 0.0
    driver = None
    last_cmd_vel_time = None
    
    # Robot wheel_track params for velocity -> motor speed conversion
    wheel_track = None
    tyre_circumference = None
    encoder_counts_per_rev = None
    m_s_to_value = 0
    axis_for_right = 0
    
    # Startup parameters
    connect_on_startup = False
    calibrate_on_startup = False
    engage_on_startup = False
    
    def __init__(self):
        self.axis_for_right = float(rospy.get_param('~axis_for_right', 0)) # if right calibrates first, this should be 0, else 1
        self.wheel_track = float(rospy.get_param('~wheel_track', 0.3)) # m, distance between wheel centres
        self.tyre_circumference = float(rospy.get_param('~tyre_circumference', 0.35)) # used to translate velocity commands in m/s into motor rpm
        
        self.connect_on_startup   = rospy.get_param('~connect_on_startup', False)
        self.calibrate_on_startup = rospy.get_param('~calibrate_on_startup', False)
        self.engage_on_startup    = rospy.get_param('~engage_on_startup', False)
        
        self.has_preroll     = rospy.get_param('~use_preroll', True)
                
        self.publish_current = rospy.get_param('~publish_current', True)
        
        self.publish_odom    = rospy.get_param('~publish_odom', True)
        self.odom_topic      = rospy.get_param('~odom_topic', "odom")
        self.odom_frame      = rospy.get_param('~odom_frame', "base_link")
        
        rospy.on_shutdown(self.terminate)

        rospy.Service('connect_driver',    std_srvs.srv.Trigger, self.connect_driver)
        rospy.Service('disconnect_driver', std_srvs.srv.Trigger, self.disconnect_driver)
    
        rospy.Service('calibrate_motors',  std_srvs.srv.Trigger, self.calibrate_motor)
        rospy.Service('engage_motors',     std_srvs.srv.Trigger, self.engage_motor)
        rospy.Service('release_motors',    std_srvs.srv.Trigger, self.release_motor)

        self.vel_subscribe = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=2)
        
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_check) # stop motors if no cmd_vel received > 1second
                
        if self.publish_current:
            self.current_loop_count = 0
            self.left_current_accumulator  = 0.0
            self.right_current_accumulator = 0.0
            self.current_timer = rospy.Timer(rospy.Duration(0.05), self.timer_current) # publish motor currents at 10Hz, read at 50Hz
            self.current_publisher_left  = rospy.Publisher('odrive/left_current', Float64, queue_size=2)
            self.current_publisher_right = rospy.Publisher('odrive/right_current', Float64, queue_size=2)
            rospy.loginfo("ODrive will publish motor currents.")
                    
        if self.publish_odom:
            self.odom_publisher  = rospy.Publisher(self.odom_topic, Odometry, queue_size=2)
            # setup message
            self.odom_msg = Odometry()
            #print(dir(self.odom_msg))
            self.odom_msg.child_frame_id = self.odom_frame
            self.odom_msg.pose.pose.position.z = 0.0    # always on the ground, we hope
            self.odom_msg.pose.pose.orientation.x = 0.0 # always vertical
            self.odom_msg.pose.pose.orientation.y = 0.0 # always vertical
            self.odom_msg.twist.twist.linear.y = 0.0  # no sideways
            self.odom_msg.twist.twist.linear.z = 0.0  # or upwards... only forward
            self.odom_msg.twist.twist.angular.x = 0.0 # or roll
            self.odom_msg.twist.twist.angular.y = 0.0 # or pitch... only yaw
            
            # store current location to be updated. 
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            
            #TODO: variables
            self.odom_timer = rospy.Timer(rospy.Duration(0.1), self.timer_odometry)
        
        if not self.connect_on_startup:
            rospy.loginfo("ODrive node started, but not connected.")
            return
        
        if not self.connect_driver(None)[0]:
            return # Failed to connect
        
        if not self.calibrate_on_startup:
            rospy.loginfo("ODrive node started and connected. Not calibrated.")
            return
        
        if not self.calibrate_motor(None)[0]:
            return
            
        if not self.engage_on_startup:
            rospy.loginfo("ODrive connected and configured. Engage to drive.")
            return
        
        if not self.engage_motor(None)[0]:
            return
        
        rospy.loginfo("ODrive connected and configured. Ready to drive.")
        
    def terminate(self):
        if self.driver:
            self.driver.release()
        self.timer.shutdown()
    
    # ROS services
    def connect_driver(self, request):
        if self.driver:
            rospy.logerr("Already connected.")
            return (False, "Already connected.")
        
        self.driver = ODriveInterfaceAPI(logger=ROSLogger())
        rospy.loginfo("Connecting to ODrive...")
        if not self.driver.connect(right_axis=self.axis_for_right):
            self.driver = False
            rospy.logerr("Failed to connect.")
            return (False, "Failed to connect.")
            
        rospy.loginfo("ODrive connected.")
        
        self.m_s_to_value = self.driver.encoder_cpr/self.tyre_circumference
                
        if self.publish_odom:
            self.old_pos_l = self.driver.left_axis.encoder.pos_cpr
            self.old_pos_r = self.driver.right_axis.encoder.pos_cpr
        
        return (True, "ODrive connected successfully")
    
    def disconnect_driver(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.disconnect():
            return (False, "Failed disconnection, but try reconnecting.")
        return (True, "Disconnection success.")
    
    def calibrate_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
            
        if self.has_preroll:
            if not self.driver.preroll():
                return (False, "Failed preroll.")        
        else:
            if not self.driver.calibrate():
                return (False, "Failed calibration.")
                
        return (True, "Calibration success.")
                    
    def engage_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
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
        if self.last_speed == 0 and abs(left_linear_val) == 0 and abs(right_linear_val) == 0:
            return
        
        # Then set your wheel speeds (using wheel_left and wheel_right as examples)
        #self.left_motor_pub.publish(left_linear_rpm)
        #self.right_motor_pub.publish(right_linear_rpm)
        #wheel_left.set_speed(v_l)
        #wheel_right.set_speed(v_r)
        
        rospy.logdebug("Driving left: %d, right: %d, from linear.x %.2f and angular.z %.2f" % (left_linear_val, right_linear_val, msg.linear.x, msg.angular.z))
        self.driver.drive(left_linear_val, right_linear_val)

        self.last_speed = max(abs(left_linear_val), abs(right_linear_val))
        self.last_cmd_vel_time = rospy.Time.now()
                
    def timer_current(self, event):
        if not self.driver or not hasattr(self.driver, 'driver') or not hasattr(self.driver.driver, 'axis0'):
            return
        
        self.left_current_accumulator += self.driver.left_axis.motor.current_control.Ibus
        self.right_current_accumulator += self.driver.right_axis.motor.current_control.Ibus
        
        self.current_loop_count += 1
        if self.current_loop_count >= 9:
            # publish appropriate axis
            self.current_publisher_left.publish(self.left_current_accumulator)
            self.current_publisher_right.publish(self.right_current_accumulator)
            
            self.current_loop_count = 0
            self.left_current_accumulator = 0.0
            self.right_current_accumulator = 0.0
        #self.current_timer = rospy.Timer(rospy.Duration(0.02), self.timer_current) # publish motor currents at 10Hz, read at 50Hz
        #self.current_publisher_left  = rospy.Publisher('left_current', Float64, queue_size=2)
        #self.current_publisher_right = rospy.Publisher('right_current', Float64, queue_size=2)
    
    def timer_odometry(self, event):
        #check for driver connected
        if self.driver is None or not self.driver.connected:
            return
        # at ~10Hz, 
        
        # poll driver for each axis position and velocity PLL estimates
        encoder_cpr = self.driver.encoder_cpr
        wheel_track = self.wheel_track   # check these. Values in m
        tyre_circumference = self.tyre_circumference
        
        vel_l = self.driver.left_axis.encoder.vel_estimate  # units: encoder counts/s
        vel_r = -self.driver.right_axis.encoder.vel_estimate # neg is forward for right
        
        # TWIST: calculated from motor values only
        self.odom_msg.twist.twist.linear.x = tyre_circumference * (vel_l+vel_r) / (2.0*encoder_cpr)
        # angle: vel_r*tyre_radius - vel_l*tyre_radius
        self.odom_msg.twist.twist.angular.z =  tyre_circumference * (vel_r-vel_l) / (wheel_track * encoder_cpr)
        self.odom_publisher.publish(self.odom_msg)
        
    def timer_check(self, event):
        """Check for cmd_vel 1 sec timeout. """
        if not self.driver:
            return
        
        if self.last_cmd_vel_time is None:
            return
        
        # if moving, and no cmd_vel received, stop
        if (event.current_real-self.last_cmd_vel_time).to_sec() > 1.0 and (self.last_speed > 0):
            rospy.logdebug("No /cmd_vel received in > 1s, stopping.")
            self.driver.drive(0,0)
            self.last_speed = 0
            self.last_cmd_vel_time = event.current_real
            

def start_odrive():
    rospy.init_node('odrive')
    odrive_node = ODriveNode()
    
    rospy.spin() 
    
if __name__ == '__main__':
    try:
        start_odrive()
    except rospy.ROSInterruptException:
        pass
