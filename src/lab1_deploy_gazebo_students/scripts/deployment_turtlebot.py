#!/usr/bin/env python
# license removed for brevity
import rospy
import actionlib
from std_msgs.msg import String
from lab1_deploy_gazebo_students.srv import gossip_update, gossip_updateResponse
from lab1_deploy_gazebo_students.msg import GoToGoal_goal, queue_position_plot
import sys
from math import radians, copysign, sqrt, pow, pi, atan2
import numpy as np
import tf

class Robot():
	def __init__(self, robot_id):
    	# Class attributes 
		self.robot_id = robot_id
		self.x = 0.0
		self.y = 0.0
		self.pub = rospy.Publisher('topic_queue_position_plot', queue_position_plot, queue_size=10)
		self.pub_goTogoal = rospy.Publisher('topic_GoToGoal_goal'+str(self.robot_id), GoToGoal_goal, queue_size=10)
		self.ser = rospy.Service('gossip_update'+str(self.robot_id), gossip_update, self.handle_gossip_update)
		self.act_dummy()

	def act_dummy(self):

		rate = rospy.Rate(1/2.0) # once every two seconds
		if (self.robot_id == 0):
			neig_id = 1
			rospy.wait_for_service('gossip_update'+str(neig_id)) # ask for service gossip_update
			try:
				print("I request a service")
				service_gossip_update = rospy.ServiceProxy('gossip_update'+str(neig_id), gossip_update)
				resp1=service_gossip_update(self.robot_id, 0.0, 1.0)
				print('Reply received: (', resp1.avg_x, ',', resp1.avg_y)

			except rospy.ServiceException as e:
				print("Service call failed: %s"%e)

			print("I publish at topic_queue_position_plot")
			my_pos_plot=queue_position_plot()
			my_pos_plot.robot_id=self.robot_id
			my_pos_plot.x=3.0
			my_pos_plot.y=4.0
			self.pub.publish(my_pos_plot)

			print("I publish a navigation goal at topicGoToGoal_goal"+str(self.robot_id))
			next_pos = GoToGoal_goal()
			next_pos.goal_coords=[-1.0, 4.0]
			next_pos.goal_z=0.0 #currently, not used
			next_pos.speed=0.0 #currently, not used
			self.pub_goTogoal.publish(next_pos)
		#Up the here, endif

		while not rospy.is_shutdown():
			hello_str = "hello world %s" % rospy.get_time()
			rospy.loginfo(hello_str)
			rate.sleep()

	def handle_gossip_update(self, req):
		# Gossip and use of bx_ij, by_ij for deploying on a line
		print("I am robot "+str(self.robot_id)+" and I received a gossip_update request: "+str(req.x)+","+str(req.y))
		myResponse=gossip_updateResponse()
		myResponse.avg_x=10.3
		myResponse.avg_y=14.6
		return myResponse


if __name__ == '__main__':
	#Input arguments (robot id, x0, y0, Tlocal, neig1, neig2... neign)
	sysargv = rospy.myargv(argv=sys.argv) # to avoid problems with __name:= elements.
	num_args=len(sysargv)
	if (num_args >= 1):
		robot_id = int(sysargv[1])
	else:
		robot_id=0
	try:
		rospy.init_node('deploy_'+str(robot_id), anonymous=False)
		my_naive_robot=Robot(robot_id)
		print('Finished!')
	except rospy.ROSInterruptException:
		pass
