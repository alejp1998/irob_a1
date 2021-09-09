#!/usr/bin/env python3
from re import purge
from sys import path
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, sqrt

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Min allowed gain to move along path (in feedback)
min_allowed_gain = 3

# Max linear velocity (m/s)
max_linear_velocity = 1
# Max angular velocity (rad/s)
max_angular_velocity = 1


def goal_active():
    rospy.loginfo("I got activated")


def goal_feedback(feedback):
    rospy.loginfo("I got feedback with gain {}".format(feedback.gain))

    # Check if this path has higher gain than min_allowed_gain
    if feedback.gain > min_allowed_gain :
        # If it has cancel goal and move along the path
        goal_client.cancel_all_goals()
        move(feedback.path)


def goal_result(state, result):
    rospy.loginfo("I got a result")

    # If the state is succeeded then
    if actionlib.TerminalState.SUCCEEDED == state:
        rospy.loginfo("Action returned succeeded")
        rospy.loginfo("I could not find a path above the threshold, exploration finished!")
        # Move one last time
        move(result.path)
        rospy.loginfo("Exploration finished :D !")

    elif actionlib.TerminalState.RECALLED == state:
        rospy.loginfo("Action returned recalled")
    elif actionlib.TerminalState.REJECTED == state:
        rospy.loginfo("Action returned rejected")
    elif actionlib.TerminalState.PREEMPTED == state:
        rospy.loginfo("Action returned preempted")
    elif actionlib.TerminalState.ABORTED == state:
        rospy.loginfo("Action returned aborted")
    elif actionlib.TerminalState.LOST == state:
        rospy.loginfo("Action returned lost")


def move(path):
    global control_client, robot_frame_id, pub
    
    while path.poses :
        # Call service client with path
        response = control_client(path)
        setpoint = response.setpoint
        new_path = response.new_path

        # Transform to change from map frame to base_link frame
        transform = tf_buffer.lookup_transform(robot_frame_id, setpoint.header.frame_id, setpoint.header.stamp)
        # Transform Setpoint from service client
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)

        # Create Twist message from the transformed Setpoint
        vel_msg = Twist()
        
        sx = transformed_setpoint.point.x
        sy = transformed_setpoint.point.y

        s_angle = atan2(sy,sx)
        s_dist = sqrt(sx**2 + sy**2)

        # Speed based on angle and distance
        th = 0.3
        alpha = 1
        beta = 0.5
        if -th < s_angle < th : 
            vel_msg.linear.x = alpha*s_dist
            vel_msg.angular.z = beta*s_angle
        else : 
            vel_msg.linear.x = beta*vel_msg.linear.x
            vel_msg.angular.z = beta*s_angle
            
        # Limit max speeds
        if vel_msg.linear.x > max_linear_velocity :
            vel_msg.linear.x = max_linear_velocity
        elif vel_msg.linear.x < -max_linear_velocity :
            vel_msg.linear.x = -max_linear_velocity
        if vel_msg.angular.z >  max_angular_velocity :
            vel_msg.angular.z = max_angular_velocity 
        elif vel_msg.angular.z < -max_angular_velocity :
            vel_msg.angular.z = -max_angular_velocity

        # Publish a message every 0.05 s
        rate = rospy.Rate(20) # 20hz

        # Publish Twist
        pub.publish(vel_msg)
        rate.sleep()

        # Call service client again if the returned path is not empty and do stuff again
        path = new_path

    # Send 0 control Twist to stop robot
    vel_msg = Twist()
    pub.publish(vel_msg)

    # Get new path from action server
    get_path()


def get_path():
    global goal_client
    goal_client.wait_for_server()
    print('Wait for goal client action server completed')

    # Get path from action server
    goal = irob_assignment_1.msg.GetNextGoalGoal()
    goal_client.send_goal(goal, active_cb=goal_active, feedback_cb=goal_feedback, done_cb=goal_result)

if __name__ == "__main__":
    # Init node
    rospy.init_node("controller")
    print('Controller node initiation completed')

    # Create TF buffer
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    print('TF Buffer and Listener initiation completed')

    # Init publisher
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    print('cmd_vel publisher initiation completed')

    # Init simple action client
    goal_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)
    print('Goal client action initiation completed')

    # Init service client
    rospy.wait_for_service('get_setpoint')
    # Handle for calling the service
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)
    print('Control client service initiation completed') 

    # Call get path
    get_path()

    # Spin
    rospy.spin()
