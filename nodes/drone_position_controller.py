#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from geometry_msgs.msg import PoseStamped

class PX4Controller:

    def __init__(self):

        # Creates a node with subscriber
        rospy.init_node('mavros_takeoff_python')
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_callback) # this is a timer to output 
        self.pose_publisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10) # for using px4 postion controller
        
        # Output a few position control setpoint so px4 allow to change into "Offboard" flight mode (i.e. to allow control code running from a companion computer)
        # Note: this can be any control setpoint (e.g. velocities, attitude , etc.)
        counter = 0
        r = rospy.Rate(50)
        while counter < 110:
            self.postion_control(0.0, 0.0, 3.0)
            counter = counter + 1
            r.sleep()

        # Now we can initialize the drone
        # self.initialize_drone()
        
        

    def control_callback(self, event):
        print 'Timer called at ' + str(event.current_real)  
        self.postion_control(0.0, 0.0, 2.0)
                                

    def postion_control(self, x, y, z): # simple position control
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.pose_publisher.publish(msg)
        

    def initialize_drone(self):

        # Set Mode
        print "\nSetting Mode"
        rospy.wait_for_service('/mavros/set_mode')
        try:
            change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = change_mode(custom_mode="OFFBOARD")
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Set mode failed: %s" %e)

        # Arm
        print "\nArming"
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arming_cl(value = True)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Arming failed: %s" %e)



if __name__ == '__main__':
    try:
        x = PX4Controller()
        rospy.spin()  
    except rospy.ROSInterruptException:
        print "error!"
