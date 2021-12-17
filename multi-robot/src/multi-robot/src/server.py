#!/usr/bin/env python
from __future__ import print_function
from geometry_msgs.msg import Point
import rospy
from gazebo_msgs.msg import ModelStates




class Server:
    def __init__(self):
        self.pos_sub=rospy.Subscriber("/person/real/position", Point, self.pos_callback, queue_size=10)
        self.gazebo_sub=rospy.Subscriber("/gazebo/model_states", ModelStates,self.gazebo_callback, queue_size=1)
        self.pos_pub1=rospy.Publisher("husky1/robot/goal", Point, queue_size=1)
        self.pos_pub2=rospy.Publisher("husky2/robot/goal", Point, queue_size=1)
        self.pos_pub3=rospy.Publisher("husky3/robot/goal", Point, queue_size=1)
        self.pos_pub4=rospy.Publisher("husky4/robot/goal", Point, queue_size=1)
        
        self.husky_state=[True,True,True,True]
        self.husky_target=[[0,0],[0,0],[0,0],[0,0]]

        self.pub_list=[self.pos_pub1,self.pos_pub2,self.pos_pub3,self.pos_pub4]
        self.person_list=[]
        self.robot_pos=[]

    def pos_callback(self, msg):
        if len(self.person_list)<1:
            robot_id=self.get_closest_robot(msg)
            self.pub_list[robot_id].publish(msg)
            self.husky_state[robot_id]=False
            self.husky_target[robot_id]=[msg.x,msg.y]
            self.person_list.append([msg.x,msg.y])
            print("[detection] total {} person detected:{},{}".format(len(self.person_list),msg.x,msg.y))

        else:
            min_dist2=10000
            for x,y in self.person_list:
                dist2=(msg.x-x)**2+(msg.y-y)**2
                if min_dist2>dist2:
                    min_dist2=dist2
            if min_dist2>16: # if minimum distance is bigger than 3m
                self.person_list.append([msg.x,msg.y])
                # self.pos_pub.publish(msg) publish the person position
                robot_id=self.get_closest_robot(msg)
                self.pub_list[robot_id].publish(msg)
                self.husky_state[robot_id]=False
                self.husky_target[robot_id]=[msg.x,msg.y]
                print("[detection] total {} person detected:{},{}".format(len(self.person_list),msg.x,msg.y))
                
        rate.sleep()

    def get_closest_robot(self, target_pos):
        dist=[]
        if len(self.robot_pos)==4:
            for i,pos in enumerate(self.robot_pos):
                if self.husky_state[i] is True:
                    dist.append((pos.x-target_pos.x)**2+(pos.y-target_pos.y)**2)
                else:
                    dist.append((4-i)*100000) # if all robots are busy
            return dist.index(min(dist)) # husky id starts from 1 to 4

    def gazebo_callback(self, data):
        self.robot_pos=[]
        for i in range(1,5): # husky starts from 1 to 4
            robot_index = data.name.index('husky{}'.format(i))
            pos = data.pose[robot_index].position
            if (pos.x-self.husky_target[i-1][0])**2+(pos.y-self.husky_target[i-1][1])**2<25:
                self.husky_state[i-1]=True
            self.robot_pos.append(pos)
            


if __name__=="__main__":
    rospy.init_node("server")
    rate=rospy.Rate(10)
    s=Server()
    rospy.spin()
