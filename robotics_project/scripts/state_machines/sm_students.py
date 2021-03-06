#!/usr/bin/env python

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class StateMachine(object):
    def __init__(self):
        
        self.node_name = "Student SM"
        self.cube_PoseStamped = PoseStamped() 

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        self.cube_pose_top = rospy.get_param(rospy.get_name() + '/cube_pose')
        self.robot_base_frame_nm = rospy.get_param(rospy.get_name() + '/robot_base_frame')   
        print("--------------------------------------------")
        print("Extract cube initial position and orientation")
        self.x_pos = float(self.cube_pose_top.split(",")[0])
        self.y_pos = float(self.cube_pose_top.split(",")[1])
        self.z_pos = float(self.cube_pose_top.split(",")[2])
        self.x_ori = float(self.cube_pose_top.split(",")[3])
        self.y_ori = float(self.cube_pose_top.split(",")[4])
        self.z_ori = float(self.cube_pose_top.split(",")[5])
        self.w_ori = float(self.cube_pose_top.split(",")[6])
        print("--------------------------------------------")

        # Subscribe to topics

        # Wait for service providers
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)
        rospy.wait_for_service(self.pick_srv_nm, timeout=30)
        rospy.wait_for_service(self.place_srv_nm, timeout=30)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.aruco_pose_pub = rospy.Publisher(self.aruco_pose_top, PoseStamped, queue_size=10)

        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

        # Init state machine
        self.state = 1 # tuck arm
        rospy.sleep(3)
        self.check_states()


    # set cube pick/place goal pose in Poststamped msg
    def set_cube_pose(self, action):
        if action == 'pick' or 'place':
            self.cube_PoseStamped.header.stamp = rospy.Time.now()
            self.cube_PoseStamped.header.frame_id = self.robot_base_frame_nm
            self.cube_PoseStamped.pose.position.x = self.x_pos
            self.cube_PoseStamped.pose.position.y = self.y_pos
            self.cube_PoseStamped.pose.position.z = self.z_pos
            self.cube_PoseStamped.pose.orientation.x = self.x_ori  
            self.cube_PoseStamped.pose.orientation.y = self.y_ori
            self.cube_PoseStamped.pose.orientation.z = self.z_ori
            self.cube_PoseStamped.pose.orientation.w = self.w_ori 
    # def set_cube_pose(self, action):
    #     if action == "pick":
    #         self.cube_PoseStamped.header.stamp = rospy.Time.now()
    #         self.cube_PoseStamped.header.frame_id = self.robot_base_frame_nm
    #         self.cube_PoseStamped.pose.position.x = 0.50306828716
    #         self.cube_PoseStamped.pose.position.y = 0.0245718046511
    #         self.cube_PoseStamped.pose.position.z = 0.915538062216
    #         self.cube_PoseStamped.pose.orientation.x = 0.0144467629456 
    #         self.cube_PoseStamped.pose.orientation.y = 0.706141958739
    #         self.cube_PoseStamped.pose.orientation.z = 0.707257659069
    #         self.cube_PoseStamped.pose.orientation.w = -0.0306827123383
    #     if action == "place":
    #         self.cube_PoseStamped.header.stamp = rospy.Time.now()
    #         self.cube_PoseStamped.header.frame_id = self.robot_base_frame_nm
    #         self.cube_PoseStamped.pose.position.x = 0.50306828716
    #         self.cube_PoseStamped.pose.position.y = 0.0245718046511
    #         self.cube_PoseStamped.pose.position.z = 0.915538062216 + 0.03
    #         self.cube_PoseStamped.pose.orientation.x = 0.0144467629456 
    #         self.cube_PoseStamped.pose.orientation.y = 0.706141958739
    #         self.cube_PoseStamped.pose.orientation.z = 0.707257659069
    #         self.cube_PoseStamped.pose.orientation.w = -0.0306827123383


    # Tuck arm => Pick cube => Go to table 2 => Place cube
    def check_states(self):

        while not rospy.is_shutdown() and self.state != 4:
            
            # State 0: Pick the cube
            if self.state == 0:
            	try:
                    rospy.loginfo("%s: Picking the cube...", self.node_name)

                    self.set_cube_pose("pick") # set cube pick goal position in Poststamped msg
                    self.aruco_pose_pub.publish(self.cube_PoseStamped) # publish cube pick goal position
                    rospy.sleep(1)

                    pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
                    pick_srv_req = pick_srv()
                    
                    if pick_srv_req.success == True:
                        rospy.loginfo("%s: Pick the cube succeded!", self.node_name)
                        self.state = 2 # Go to table 2
                    else:
                        rospy.loginfo("%s: Pick the cube failed!", self.node_name)
                        self.state = 5

                    rospy.sleep(3)
                
                except rospy.ServiceException, e:
                    print "Service call to pick cube server failed: %s"%e

            # State 1:  Tuck arm 
            if self.state == 1:
                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))

                if success_tucking:
                    rospy.loginfo("%s: Arm tuck: ", self.node_name)
                    self.state = 0 # pick the cube 
                else:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    self.state = 5

                rospy.sleep(1)

            # State 2:  Move the robot "manually" to table2
            if self.state == 2:
                move_msg = Twist()
                move_msg.angular.z = -0.5

                rate = rospy.Rate(10)
                converged = False
                cnt = 0
                rospy.loginfo("%s: Moving towards table", self.node_name)
                while not rospy.is_shutdown() and cnt < 60:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1
                rospy.sleep(1)

                move_msg.linear.x = 1
                move_msg.angular.z = 0
                cnt = 0
                while not rospy.is_shutdown() and cnt < 10:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                self.state = 3 # place the cube 
                rospy.sleep(1)

            # State 3:  Lower robot head service
            if self.state == 3:
            	try:
                    rospy.loginfo("%s: Placing the cube...", self.node_name)

                    self.set_cube_pose("place") # set cube pick goal position in Poststamped msg
                    self.aruco_pose_pub.publish(self.cube_PoseStamped) # publish cube pick goal position
                    rospy.sleep(1)

                    place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)
                    place_srv_req = place_srv()
                    
                    if place_srv_req.success == True:
                        rospy.loginfo("%s: Place the cube succeded!", self.node_name)
                        self.state = 4 # sm finish
                    else:
                        rospy.loginfo("%s: Place the cube failed!", self.node_name)
                        self.state = 5

                    rospy.sleep(1)
                
                except rospy.ServiceException, e:
                    print "Service call to place cube server failed: %s"%e

            # Error handling
            if self.state == 5:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return


# import py_trees as pt, py_trees_ros as ptr

# class BehaviourTree(ptr.trees.BehaviourTree):

# 	def __init__(self):

# 		rospy.loginfo("Initialising behaviour tree")

# 		# go to door until at door
# 		b0 = pt.composites.Selector(
# 			name="Go to door fallback", 
# 			children=[Counter(30, "At door?"), Go("Go to door!", 1, 0)]
# 		)

# 		# tuck the arm
# 		b1 = TuckArm()

# 		# go to table
# 		b2 = pt.composites.Selector(
# 			name="Go to table fallback",
# 			children=[Counter(5, "At table?"), Go("Go to table!", 0, -1)]
# 		)

# 		# move to chair
# 		b3 = pt.composites.Selector(
# 			name="Go to chair fallback",
# 			children=[Counter(13, "At chair?"), Go("Go to chair!", 1, 0)]
# 		)

# 		# lower head
# 		b4 = LowerHead()

# 		# become the tree
# 		tree = pt.composites.Sequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
# 		super(BehaviourTree, self).__init__(tree)

# 		# execute the behaviour tree
# 		self.setup(timeout=10000)
# 		while not rospy.is_shutdown(): self.tick_tock(1)


# class Counter(pt.behaviour.Behaviour):

# 	def __init__(self, n, name):

# 		# counter
# 		self.i = 0
# 		self.n = n

# 		# become a behaviour
# 		super(Counter, self).__init__(name)

# 	def update(self):

# 		# count until n
# 		while self.i <= self.n:

# 			# increment count
# 			self.i += 1

# 			# return failure :(
# 			return pt.common.Status.FAILURE

# 		# succeed after counter done :)
# 		return pt.common.Status.SUCCESS


# class Go(pt.behaviour.Behaviour):

# 	def __init__(self, name, linear, angular):

# 		# action space
# 		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
# 		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

# 		# command
# 		self.move_msg = Twist()
# 		self.move_msg.linear.x = linear
# 		self.move_msg.angular.z = angular

# 		# become a behaviour
# 		super(Go, self).__init__(name)

# 	def update(self):

# 		# send the message
# 		rate = rospy.Rate(10)
# 		self.cmd_vel_pub.publish(self.move_msg)
# 		rate.sleep()

# 		# tell the tree that you're running
# 		return pt.common.Status.RUNNING


# class TuckArm(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# Set up action client
# 		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

# 		# personal goal setting
# 		self.goal = PlayMotionGoal()
# 		self.goal.motion_name = 'home'
# 		self.goal.skip_planning = True

# 		# execution checker
# 		self.sent_goal = False
# 		self.finished = False

# 		# become a behaviour
# 		super(TuckArm, self).__init__("Tuck arm!")

# 	def update(self):

# 		# already tucked the arm
# 		if self.finished: 
# 			return pt.common.Status.SUCCESS
		
# 		# command to tuck arm if haven't already
# 		elif not self.sent_goal:

# 			# send the goal
# 			self.play_motion_ac.send_goal(self.goal)
# 			self.sent_goal = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# if I was succesful! :)))))))))
# 		elif self.play_motion_ac.get_result():

# 			# than I'm finished!
# 			self.finished = True
# 			return pt.common.Status.SUCCESS

# 		# if I'm still trying :|
# 		else:
# 			return pt.common.Status.RUNNING
		


# class LowerHead(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# server
# 		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
# 		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
# 		rospy.wait_for_service(mv_head_srv_nm, timeout=30)

# 		# execution checker
# 		self.tried = False
# 		self.tucked = False

# 		# become a behaviour
# 		super(LowerHead, self).__init__("Lower head!")

# 	def update(self):

# 		# try to tuck head if haven't already
# 		if not self.tried:

# 			# command
# 			self.move_head_req = self.move_head_srv("down")
# 			self.tried = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# react to outcome
# 		else: return pt.common.Status.SUCCESS if self.move_head_req.success else pt.common.Status.FAILURE


	

if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		#StateMachine()
		StateMachine()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
