#!/usr/bin/env python
from __future__ import print_function

#import roslib; roslib.load_manifest('BINCADDY')
import rospy
import tf.transformations

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

import time

from odrive_interface import ODriveInterfaceSerial, ODriveInterfaceAPI

class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def debug(self, msg):    rospy.logdebug(msg)
    def info(self, msg):     rospy.loginfo(msg) 
    def warn(self, msg):     rospy.logwarn(msg) 
    def error(self, msg):    rospy.logerr(msg)  
    def critical(self, msg): rospy.logfatal(msg)
    

# TODO: map to ROS parameters
wheelbase = 0.3             # m, distance between wheel centres
tyre_circumference = 0.35    # used to translate velocity commands in m/s into motor rpm

#m_s_to_rpm = 60.0/tyre_circumference
#m_s_to_erpm = 10 * m_s_to_rpm 

# 4096 counts / rev, so 4096 == 1 rev/s
m_s_to_value = 4096/tyre_circumference

# 1 m/s = 3.6 km/hr

def convert(forward, ccw):
    angular_to_linear = ccw * (wheelbase/2.0) 
    left_linear_val  = int((forward - angular_to_linear) * m_s_to_value)
    right_linear_val = int((forward + angular_to_linear) * m_s_to_value)
    
    return left_linear_val, right_linear_val

class ODriveNode(object):
    last_speed = 0.0
    driver = None
    last_cmd_vel_time = None
    
    def __init__(self):
        self.driver = ODriveInterfaceAPI(logger=ROSLogger())
        rospy.loginfo("Connecting...")
        self.driver.connect('/dev/ttyACM0')
        #    import sys; sys.exit(1)
            
        self.driver.setup()
        self.driver.engage()
                
        # /commands/motor/brake
        # /commands/motor/current
        # /commands/motor/duty_cycle
        # /commands/motor/position
        # /commands/motor/speed
        # /commands/servo/position
        
        #odrv0.axis0.motor.current_meas_phC 
        
        self.vel_subscribe = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=2)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_check) # stop motors if no cmd_vel received > 1second
        
        rospy.loginfo("ODrive connected and configured. Ready to drive.")
        
    def terminate(self):
        self.driver.release()
        self.timer.shutdown()

    def cmd_vel_callback(self, msg):
        #rospy.loginfo("Received a /cmd_vel message!")
        #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

        # rostopic pub -r 1 /commands/motor/current std_msgs/Float64 -- -1.0

        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities into motor commands
        
        # 3600 ERPM = 360 RPM ~= 6 km/hr
        
        #angular_to_linear = msg.angular.z * (wheelbase/2.0) 
        #left_linear_rpm  = (msg.linear.x - angular_to_linear) * m_s_to_erpm
        #right_linear_rpm = (msg.linear.x + angular_to_linear) * m_s_to_erpm
        left_linear_val, right_linear_val = convert(msg.linear.x, msg.angular.z)
        
        # if wheel speed = 0, stop publishing after sending 0 once. #TODO add error term, work out why VESC turns on for 0 rpm
        if self.last_speed == 0 and abs(left_linear_val) == 0 and abs(right_linear_val) == 0:
            return
        
        # Then set your wheel speeds (using wheel_left and wheel_right as examples)
        #self.left_motor_pub.publish(left_linear_rpm)
        #self.right_motor_pub.publish(right_linear_rpm)
        #wheel_left.set_speed(v_l)
        #wheel_right.set_speed(v_r)
        rospy.logdebug("Driving left: %d, right: %d, from linear.x %.2f and angular.z %.2f" % (left_linear_val, right_linear_val, msg.linear.x, msg.angular.z))
        self.driver.drive(-left_linear_val, right_linear_val)

        self.last_speed = max(abs(left_linear_val), abs(right_linear_val))
        self.last_cmd_vel_time = rospy.Time.now()
        
    def timer_check(self, event):
        """Check for cmd_vel 1 sec timeout. """
        if self.last_cmd_vel_time is None:
            return
        
        # if moving, and no cmd_vel received, stop
        if (event.current_real-self.last_cmd_vel_time).to_sec() > 1.0 and (self.last_speed > 0):
            rospy.logdebug("No /cmd_vel received in > 1s, stopping.")
            self.driver.drive(0,0)
            self.last_cmd_vel_time = event.current_real
            

def start_odrive():
    rospy.init_node('odrive')
    odrive_node = ODriveNode()
    rospy.on_shutdown(odrive_node.terminate)
    
    rospy.spin() 
    
if __name__ == '__main__':
    try:
        start_odrive()
    except rospy.ROSInterruptException:
        pass
