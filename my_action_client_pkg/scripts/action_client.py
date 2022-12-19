#! /usr/bin/env python

import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

nImage = 1

def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d received' %nImage)
    nImage += 1

def set_connections(pub):
    connections = pub.get_num_connections()
    while connections < 1:
        connections = pub.get_num_connections()
        rate.sleep()

rospy.init_node('drone_action_client')
rate = rospy.Rate(1)

client = actionlib.SimpleActionClient('/ardrone_action_server', ArdroneAction)
client.wait_for_server()

# connect to takeoff topic 
pub = rospy.Publisher("/drone/takeoff", Empty, queue_size=1)
set_connections(pub)
empty = Empty()
pub.publish(empty)


# use topic to move the drone
movement_msg = Twist()
movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
set_connections(movement_pub)
movement_msg.linear.x = 0.5
movement_msg.angular.z = 0.5
movement_pub.publish(movement_msg)

# initialize and send a goal
goal = ArdroneGoal()
goal.nseconds = 10
client.send_goal(goal, feedback_cb=feedback_callback)

state_result = client.get_state()

rospy.loginfo("state_result: "+str(state_result))

while state_result < DONE:
    rospy.loginfo("Doing Stuff while waiting for the Server to give a result....")
    rate.sleep()
    state_result = client.get_state()
    rospy.loginfo("state_result: "+str(state_result))

# stop the drone using topic
set_connections(movement_pub)
movement_msg.linear.x = 0
movement_msg.angular.z = 0
movement_pub.publish(movement_msg)

# connect to land topic
land_pub = rospy.Publisher("/drone/land", Empty, queue_size=1)
set_connections(land_pub)
land_pub.publish(empty)

print('[Result] State: %d' %(client.get_state()))
if state_result == ERROR:
    rospy.logerr("Something went wrong in the Server Side")
if state_result == WARN:
    rospy.logwarn("There is a warning in the Server Side")