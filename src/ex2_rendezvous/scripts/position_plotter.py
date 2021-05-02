#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
#from lab1_deploy_gazebo_students.srv import *
from lab1_deploy_gazebo_students.msg import *
import sys
from math import radians, copysign, sqrt, pow, pi, atan2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib import animation
matplotlib.use("TkAgg")

class Plotter():
	def __init__(self, n_robots):
    	# Class attributes 
		self.n_robots = n_robots
        # Positions of all the robots. To make things easier, we assume that the robot ids are consecutive 0..n-1 
		# and that here we index them in the array with 0..n-1 respectively.
		self.v_robot_x = np.zeros([n_robots])
		self.v_robot_y = np.zeros([n_robots])
		self.sub = rospy.Subscriber('topic_queue_position_plot', queue_position_plot, self.callback_position_received)
		self.act_drawing()

	def act_drawing(self):		
		rospy.sleep(0.1)# give some time to receive at least a set of robot positions.
		while not rospy.is_shutdown():
			#print('vamos')
			plt.clf()
			#plt.xlim(0,15)
			#plt.ylim(0,15)
			for i in range(0,n_robots):
				plt.plot(self.v_robot_x[i],self.v_robot_y[i],'bo')
			plt.draw()
			plt.show(block=False)
			plt.pause(0.01)
			rospy.sleep(0.1) # ten times per second. You can increase this rate

	def callback_position_received(self, new_position):
		print(rospy.get_caller_id() + "I heard:")
		print('robot_id: ', new_position.robot_id)
		#print('x: ', new_position.x)
		#print('y: ', new_position.y)
		#self.v_robot_x[new_position.robot_id-1]=new_position.x
		#self.v_robot_y[new_position.robot_id-1]=new_position.y
		self.v_robot_x[new_position.robot_id]=new_position.x
		self.v_robot_y[new_position.robot_id]=new_position.y
		print('Positions printer x : ', self.v_robot_x, 'Positions y: ', self.v_robot_y)

if __name__ == '__main__':
	#Input arguments (robot id, x0, y0, Tlocal, neig1, neig2... neign)
	sysargv = rospy.myargv(argv=sys.argv) # to avoid problems with __name:= elements.
	num_args=len(sysargv)
	if (num_args >= 1):
		n_robots = int(sysargv[1])
	else:
		n_robots = 1
	try:
		rospy.init_node('position_plotter', anonymous=False)
		my_plotter=Plotter(n_robots)
		print('Hello World!')
		print('Printing the positions of ', my_plotter.n_robots)
	except rospy.ROSInterruptException:
		pass
