#!/usr/bin/env python
# license removed for brevity
import rospy
import actionlib
from std_msgs.msg import String
from ex2_rendezvous.srv import gossip_update, gossip_updateResponse
from ex2_rendezvous.msg import queue_position_plot
import sys
from math import radians, copysign, sqrt, pow, pi, atan2
import numpy as np
import tf

class Robot():
    def __init__(self, robot_id, x0, y0, Tlocal, robots_ids):
        # Class attributes 
        self.robot_id = robot_id
        self.x = x0
        self.y = y0
        self.pub = rospy.Publisher('topic_queue_position_plot', queue_position_plot, queue_size=10)
        self.set_neig(robots_ids)
        self.ser = rospy.Service('gossip_update'+str(self.robot_id), gossip_update, self.handle_gossip_update)
        print(self.neig_id)
        self.act_dummy(Tlocal)

    def set_neig(self, robots_ids):
        count = 0
        ne_tmp = -1
        if not np.amax(robots_ids) <= self.robot_id:
            while ne_tmp <= self.robot_id and count < 10000:
                r = np.random.randint(0, len(robots_ids))
                ne_tmp = robots_ids[r]
        self.neig_id = ne_tmp

    def act_dummy(self, Tlocal):

        rate = rospy.Rate(Tlocal)
        print(Tlocal)

        while not rospy.is_shutdown():
            if (self.neig_id > -1):
                rospy.wait_for_service('gossip_update'+str(self.neig_id)) # ask for service gossip_update
                try:
                    print("I request a service")
                    service_gossip_update = rospy.ServiceProxy('gossip_update'+str(self.neig_id), gossip_update)
                    resp1=service_gossip_update(self.robot_id, self.x, self.y)
                    print('Reply received: (', resp1.avg_x, ',', resp1.avg_y)
                    self.x = resp1.avg_x
                    self.y = resp1.avg_y

                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
            else:
                self.x += 0
                self.y += 0
            my_pos_plot=queue_position_plot()
            my_pos_plot.robot_id=self.robot_id
            my_pos_plot.x=self.x
            my_pos_plot.y=self.y
            self.pub.publish(my_pos_plot)
            rospy.loginfo(my_pos_plot)
            
            print("I publish a navigation goal at topicGoToGoal_goal"+str(self.robot_id))
            next_pos = GoToGoal_goal()
            next_pos.goal_coords=[-1.0, 4.0]
            next_pos.goal_z=1.0 #currently, not used
            next_pos.speed=1.0 #currently, not used
            self.pub_goTogoal.publish(next_pos)
            rate.sleep()

    def handle_gossip_update(self, req):
        # Gossip and use of bx_ij, by_ij for deploying on a line
        print("I am robot "+str(self.robot_id)+" and I received a gossip_update request: "+str(req.x)+","+str(req.y))
        myResponse=gossip_updateResponse()
        myResponse.avg_x=((self.x + (self.robot_id - req.robot_id)*3)*0.2 + req.x*0.8)
        myResponse.avg_y=(self.y*0.2 + req.y*0.8)
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
        robots_ids = [ int(arg_str) for arg_str in sysargv[5:] ]
        print(robots_ids)
        my_naive_robot=Robot(int(robot_id), float(sysargv[2]), float(sysargv[3]), int(sysargv[4]), robots_ids)
        print('Finished!')
    except rospy.ROSInterruptException:
        pass
