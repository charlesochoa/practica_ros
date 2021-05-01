#! /usr/bin/env python
# coding=utf-8

import rospy
import actionlib
from lab1_deploy_gazebo_students.msg import GoToGoal_goal
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import sys
import rosnode

# Script in charge of moving every robot towards its goal position. It is quite simple! 
# It has one input argument: the robot identifier 


def get_odom(base_frame, odom_frame, tf_listener):
   (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
   rotation = euler_from_quaternion(rot)
   return (Point(*trans), rotation[2])

def get_goal_distance_angle(goal_x, goal_y, pos_x, pos_y, rotation):
	# in case rotation in a different range than atan2
	#print('Rotation: ', rotation*180/np.pi)
	rot=rotation
	if rotation < -np.pi:
		rot=rotation+2*np.pi
	elif rotation > np.pi:
		rot=rotation-2*np.pi
	rot_plus2pi = rot + 2*np.pi
	rot_minus2pi = rot - 2*np.pi
	#print('rot: ', rot*180/np.pi, ', rot_plus2pi:', rot_plus2pi*180/np.pi, ', rot_minus2pi:', rot_minus2pi*180/np.pi)

	# distance to goal
	goal_distance = sqrt(pow(goal_x-pos_x, 2) + pow(goal_y-pos_y, 2))
    # angle to goal
	path_angle = atan2(goal_y-pos_y, goal_x-pos_x)
	#print('path_angle: ', path_angle*180/np.pi)

    # To select the smallest rotation angle
	w1 = path_angle-rot
	w2 = path_angle-rot_plus2pi
	w3 = path_angle-rot_minus2pi
	w_all = np.array([w1, w2, w3])
	ind_min = np.argmin(abs(w_all))
	w=w_all[ind_min]
	#print('w: ', w*180/np.pi, ', w1:', w1*180/np.pi, ', w2:', w2*180/np.pi, ', w3:', w3*180/np.pi)

	return goal_distance, w

class RobotGoToGoalManager():

	def __init__(self, robotnumber):
		self.robotnumber = robotnumber
		self.manager_name = "tb3_"+self.robotnumber+"_as"
		self.goal_x=0.0
		self.goal_y=0.0
		self.goal_achieved = True # to move the robot only if the current goal has not been achieved yet
		#self.goal_achieved = False # Trick to move the robot to (0,0) as it starts

		self.tolerance = 0.3 # stop at 0.3 meters from destination
		self.linear_speed = 1
		self.angular_speed = 2 # aim: that the angular speed is large
		

        #self.a_server = actionlib.SimpleActionServer(
        #    self.manager_name, GoToGoalAction, execute_cb=self.execute_cb, auto_start=False)
        #self.a_server.start()
		self.tf_listener = tf.TransformListener()
		self.vel_topic = "/tb3_"+self.robotnumber+"/cmd_vel"
		self.base_frame = "/tb3_"+self.robotnumber+"/base_footprint"
		self.odom_frame = "/tb3_"+self.robotnumber+"/odom"

		self.sub_goals = rospy.Subscriber('topic_GoToGoal_goal'+self.robotnumber, GoToGoal_goal, self.callback_goal_received)

		self.cmd_vel = rospy.Publisher(self.vel_topic, Twist, queue_size=5)

		self.act_GoToGoal()

	def act_GoToGoal(self):

        # To get the current position from Gazebo (ground truth data)
		try:
			self.tf_listener.waitForTransform(
                self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(3.0))
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			try:
				self.tf_listener.waitForTransform(
                    self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(3.0))
				base_frame = 'base_link'
			except (tf.Exception, tf.ConnectivityException, tf.LookupException):
				rospy.loginfo(
                    "Cannot find transform between odom and base_link or base_footprint")
				rospy.signal_shutdown("tf Exception")
       
		position = Point()
		move_cmd = Twist()
		r = rospy.Rate(1)
		
		while not rospy.is_shutdown():
			# ground truth position and rotation
			(pos, rot) = get_odom(self.base_frame, self.odom_frame, self.tf_listener)
			#print('Current position: ', pos, ' and orientation (degrees): ', rot*180/np.pi)

			if not(self.goal_achieved): # then move towards goal
				# from the current pos and ori, distance to traverse, and orientation to rotate
				goal_distance, goal_angle= get_goal_distance_angle(self.goal_x, self.goal_y, pos.x, pos.y, rot)	
				#print('Angle: ', path_angle*180/np.pi)

				if goal_distance > self.tolerance: #move towards the goal
					# speeds: propotional to the error, with saturation
					move_cmd.angular.z = self.angular_speed * goal_angle
					if move_cmd.angular.z > 0:
						move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
					else:
						move_cmd.angular.z = max(move_cmd.angular.z, -1.5)
					move_cmd.linear.x = min(self.linear_speed * goal_distance, 0.3)
					
				else: # stop robot
					self.goal_achieved=True
					rospy.loginfo("Goal achieved. Stop robot...")					
					move_cmd.linear.x=0.0
					move_cmd.angular.z=0.0
			else:
				move_cmd.linear.x=0.0
				move_cmd.angular.z=0.0

			#print(move_cmd)
			self.cmd_vel.publish(move_cmd)			
			r.sleep()

	def callback_goal_received(self, new_position_goal):
		# When a new goal is received, the robot gets active and starts moving towards it
		# In case it was already moving towards a different goal, it just "cancels" this goal 
		# and moves towards the new one. Feel free to modify this / to use actionlib instead
		print(rospy.get_caller_id() + "I received a new goal")

		self.goal_x=new_position_goal.goal_coords[0]
		self.goal_y=new_position_goal.goal_coords[1]
		self.goal_achieved = False # to move the robot only if the current goal has not been achieved yet
		# Additional elements that could be used if it is desired:
		#new_position_goal.goal_z # defined, not used in this version
		#new_position_goal.speed # defined, not used in this version

		print('New goal, x : ', self.goal_x, ', y: ', self.goal_y)


if __name__ == "__main__":
	if len(sys.argv) >= 2:
		robot_number = str(sys.argv[1])
		print("Robot controlled: " + robot_number)
	else:
		print("Enter the robot id to be managed") # example interactive
		robot_number = str(raw_input("Number of robot\n"))
		pass

	rospy.init_node("tb3_"+str(robot_number)+"_as")
	s = RobotGoToGoalManager(robot_number)
	#print('pruebas!!!')
	#goal_x=0
	#goal_y=0
	#pos_x=-2
	#pos_y=0
	#rotation=np.pi
	#get_goal_distance_angle(goal_x, goal_y, pos_x, pos_y, rotation)



