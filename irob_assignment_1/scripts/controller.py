#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot


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
# Max linear velocity (m/s), moving velocity
max_linear_velocity = 0.3 #0.5
# Max angular velocity (rad/s), turning velocity
max_angular_velocity = 2.0 #1.0


def move(path):
    global control_client, robot_frame_id, pub
    new_path = path

    rate = rospy.Rate(20) 
    # Call service client again if the returned path is not empty and do stuff again
    while new_path.poses:
    
        # Call service client with path
        setpoint = control_client(new_path).setpoint
        new_path = control_client(new_path).new_path
        #rospy.loginfo("The frame is: %s", setpoint.header.frame_id)

        # Transform Setpoint from service client to base_link
        transform = tf_buffer.lookup_transform(robot_frame_id, setpoint.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform).point
        #print('CHECK3')

        # Create Twist message from the transformed Setpoint
        msg = Twist()
        msg.angular.z = max_angular_velocity * atan2(transformed_setpoint.y, transformed_setpoint.x)
        msg.linear.x = max_linear_velocity * hypot(transformed_setpoint.x, transformed_setpoint.y)
        
        # Publish Twist on /cmd_vel topic
        pub.publish(msg)
        rate.sleep()

    # Send 0 control Twist to stop robot
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub.publish(msg)

    # Get new path from action server
    print('CEHCK4')
    get_path()


def get_path():
    global goal_client
    print('CHECK5')

    # Get path from action server
    # Waits until the action server has started up and started listening for goals.
    goal_client.wait_for_server()
    
    # Creates a goal to send to the action server.
    #goal = irob_assignment_1.msg.GetNextGoalAction()

    # Sends the goal to the action server.
    #goal_client.send_goal(goal)
    goal_client.send_goal(None)

    # Waits for the server to finish performing the action.
    goal_client.wait_for_result()
    # Get the result of executing the action
    path = goal_client.get_result().path
    print('CHECK1')
    
    # Call move with path from action server
    while path.poses: 
        print('CHECK2')
        move(path)


if __name__ == "__main__":
    # Init node: define node name as "controller", then rospy can communicate with ROS Master
    rospy.init_node("controller")

    # Init publisher: pub,  which publish on 'cmd_vel' topic whose type is Twist
    # queue_size limits amount of queued message to 10 for subscribers to receive messages in affordable time
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Init simple action client: goal_client
    # communicate with server over topic "get_next_goal" whose type is GetNextGoalAction
    goal_client = actionlib.SimpleActionClient("get_next_goal", irob_assignment_1.msg.GetNextGoalAction)
    
    # Init service client
    control_client = rospy.ServiceProxy("get_setpoint", GetSetpoint)

    # create TF buffer, a subscriber receiving frame transform information
    tf_buffer = tf2_ros.Buffer() # constructor for a buffer object
    listener = tf2_ros.TransformListener(tf_buffer)

    # Call get path
    get_path()
    
    # Spin
    rospy.spin()
