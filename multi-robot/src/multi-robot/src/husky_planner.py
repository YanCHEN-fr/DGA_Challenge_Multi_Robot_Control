#!/usr/bin/env python

'''
Plan husky robot from one position to another position
'''

from __future__ import print_function
import cv2
import numpy as np
import time
import math
# from matplotlib import pyplot as plt
# plt.switch_backend('agg')
import rospy
from geometry_msgs.msg import Twist, Point, Pose
from gazebo_msgs.msg import ModelStates
import cv_bridge
from sensor_msgs.msg import Image
import random


def angle_wrap(a):
    ## limit the angle between -pi and pi ##

    while (a > math.pi):
        a = a - 2 * math.pi
    while (a <= -math.pi):
        a = a + 2 * math.pi

    return a


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


class movement(object):
    def __init__(self, robot_N):
        self.position = np.array([0, 0, 0, 0])
        
        self.rate = rospy.Rate(10)
        self.velocity_publisher2 = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()
        self.robot_N = robot_N
        self.track_flag = False
        self.default_pose_flag = True
        self.count = 0
        self.wait = True
        self.isnode = False
        self.goals=[]
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/depth/image_raw', Image, self.image_callback)
        rospy.Subscriber('robot/position', Pose,self.pose_callback)
        self.target_sub=rospy.Subscriber("robot/goal",Point,self.target_callback)

    def target_callback(self, point):
        self.goals.append([point.x,point.y])
        self.isnode=True
        print("{} get person information".format(self.robot_N))

    def pose_callback(self, pose_msg):
        x=pose_msg.position.x
        y=pose_msg.position.y
        z=pose_msg.position.z

        rotq = pose_msg.orientation
        angle = euler_from_quaternion(rotq.x, rotq.y, rotq.z, rotq.w)
        self.position = np.array([x, y, z, angle[2]])


    def image_callback(self, msg):
        # BEGIN BRIDGE
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.left = 0
        self.close = 0
        width = self.image.shape[1]
        detection_width = 300

        start_row = 200
        end_row = 300

        window = self.image[255, width/2-detection_width/2:width/2+detection_width/2]
        index = np.argwhere(~np.isnan(window)).flatten()
        if len(index) >0 and 0.2 < window[index].min() < 1:
            self.collison = True
            left_region = np.isnan(self.image[start_row:end_row, :width // 2]).sum()
            right_region = np.isnan(self.image[start_row:end_row, width // 2:]).sum()
            if left_region > right_region:  # black area
                self.left = 1
            else:
                self.left = -1
        else:
            self.collison = False

        # too close to obstacle detection
        # if np.isnan(self.image[360][width/2-detection_width/2:width/2+detection_width/2]).sum() == detection_width:
        #     self.close = 0
        if np.isnan(self.image[360][width / 2 - detection_width / 2:width / 2 + detection_width / 2]).sum() >= 1 \
                or self.image[255][width / 2 - detection_width / 2:width / 2 + detection_width / 2].min() <= 0.2:
            self.close = 0
        else:
            self.close = 1

        # cv2.namedWindow("robot_camera_window", 1)
        # cv2.imshow("robot_camera_window", self.image)
        # cv2.waitKey(1)

    def update_velocity(self, v_x, v_y, v_z, av_x, av_y, av_z):
        self.vel_msg.linear.x = v_x
        self.vel_msg.linear.y = v_y
        self.vel_msg.linear.z = v_z
        self.vel_msg.angular.x = av_x
        self.vel_msg.angular.y = av_y
        self.vel_msg.angular.z = av_z
        self.velocity_publisher2.publish(self.vel_msg)
        self.rate.sleep()

'''
 author: yowlings 
 modified by David Filliat and Yan CHEN
'''
class pathfind(movement):
    """
    Path finding in a occupancy grid map using AStar algorithms
    """
    def __init__(self, robot_N):
        super(pathfind, self).__init__(robot_N)
        self.map_file = '/home/ubuntu/dga_project/src/multi-robot/src/multi-robot/src/map.png'
        self.robot_radius = 5
        self.result_img = cv2.imread(self.map_file)
        self.map = cv2.imread(self.map_file, cv2.IMREAD_GRAYSCALE)
        self.columns, self.rows = self.map.shape
        self.threshold()
        self.inflate()
        self.dis = cv2.distanceTransform(self.map, cv2.DIST_L2, 3) # calculate thess distances bewteen points and points 0 in the map
        self.dis_cost = self.dis.max()/(self.dis+1e-5) # thes points neaby the obstacls have more cost
        self.start = self.position[0:2]
        # self.scale = 0.118  # scale 1 pixel -> 0.118 meters
        self.scale = 0.1723
        self.org = np.array([299.4, 367.4])  # pixel coordinate of original point
        self.show_animation = False
        self.heuristic_weight = 3.0
        self.d_subsample = 2


    def threshold(self):
        """
        Binarize the obstacle map to 0/255
        """
        for i in range(self.columns):
            for j in range(self.rows):
                if self.map[i,j]>128:
                    self.map[i,j]=255
                else:
                    self.map[i,j]=0


    def inflate(self):
        """
        Inflate the obstacle size to take the robot radius into account
        """
        old_map = self.map.copy()

        for i in range(9, self.columns-10):
            for j in range(9, self.rows-10):
                if old_map[i,j]==0:
                    for u in range(-self.robot_radius, self.robot_radius, 1):
                        for v in range(-self.robot_radius, self.robot_radius, 1):
                            if(u!=0 and v!=0):
                                if (((i+u)>=0)&((i+u)<self.columns-10)&((j+v)>=0)&((j+v)<self.rows-10)):
                                    self.result_img[i+u,j+v,1]=128
                                    self.map[i+u,j+v]=256


    def get_neighbors(self,current):
        """
        Return neighbor nodes using 8-connectivity
        """
        neighbors = []
        if(current[0]+1<self.columns and self.map[current[0]+1,current[1]]==255):
            neighbors.append((current[0]+1, current[1]))
        if(current[0]-1>=0 and self.map[current[0]-1,current[1]]==255):
            neighbors.append((current[0]-1, current[1]))
        if(current[1]-1>=0 and self.map[current[0],current[1]-1]==255):
            neighbors.append((current[0], current[1]-1))
        if(current[1]+1<self.rows and self.map[current[0],current[1]+1]==255):
            neighbors.append((current[0], current[1]+1))
        if(current[0]+1<self.columns and current[1]+1<self.rows and self.map[current[0]+1,current[1]+1]==255):
            neighbors.append((current[0]+1, current[1]+1))
        if(current[0]-1>=0 and current[1]-1>=0 and self.map[current[0]-1,current[1]-1]==255):
            neighbors.append((current[0]-1, current[1]-1))
        if(current[1]-1>=0 and current[0]+1<self.columns and self.map[current[0]+1,current[1]-1]==255):
            neighbors.append((current[0]+1, current[1]-1))
        if(current[1]+1<self.rows and current[0]-1>=0 and self.map[current[0]-1,current[1]+1]==255):
            neighbors.append((current[0]-1, current[1]+1))

        return neighbors


    def heuristic(self,a,b):
        """
        Return heuristic for AStar
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])


    def draw_path(self, start, goal):
        """
        Draw the computed path, after AStar
        """
        cv2.circle(self.result_img, (start[1],start[0]), 2, (255,0,0),2)
        cv2.circle(self.result_img, (goal[1],goal[0]), 2, (0,0,255),2)

        for current in self.path:
            cv2.circle(self.result_img, (current[0],current[1]), 0, (255,0,0),0)

        return self.result_img

    def get_path_length(self):
        """
        Return path length, after AStar
        """
        length = 0
        for i,k in zip(self.path[0::], self.path[1::]):
            # length += math.dist(i,k)
            length += np.linalg.norm(np.array(i)-np.array(k))
        return length


    def transform_to_pixel(self, x):
        x = np.array(x)
        if x.ndim==1:
            x = np.array([x])/self.scale
        x[:,1] = -x[:,1]
        x = (x+self.org).astype(np.int32)
        if len(x)==1:
            return x[0]
        return x

    def trasform_to_real(self,x):
        x = np.array(x)
        if x.ndim==1:
            x = np.array([x])
        x = (x - self.org) * self.scale
        x[:, 1] = -x[:, 1]
        if len(x)==1:
            return x[0]
        return x

    def astar(self, start, goal):
        """
        Compute shortest path using AStar algorithm
        """
  
        frontier = {}
        frontier[start] = self.heuristic_weight * self.heuristic(goal, start)

        self.came_from = {}
        cost_so_far = {}
        closed = {}
        self.came_from[start] = None
        cost_so_far[start] = 0
        iter = 0

        while frontier:
            iter += 1

            current = min(frontier, key=frontier.get)
            # draw frontier for results

            self.result_img[current[0],current[1], 1] = 255

            if current == goal:
                break

            neighbors = self.get_neighbors(current)

            for next in neighbors:
                # new_cost = cost_so_far[current] + math.dist(current, next)
                new_cost = cost_so_far[current] + np.linalg.norm(np.array(current)-np.array(next))
                # new_cost = cost_so_far[current] + math.dist(current, next) + self.dis_cost[next]
                test = (next in closed) or ((next in frontier) and (new_cost > cost_so_far[next]))
                if not test:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic_weight * self.heuristic(goal, next)
                    frontier[next] = priority
                    self.came_from[next] = current

                    # draw frontier for results
                    self.result_img[next[0],next[1],0] = 0
                    self.result_img[next[0],next[1],1] = 0
            closed[current] = 1
            frontier.pop(current)

            # if self.show_animation and iter % 100 ==0:
            #     plt.imshow(self.result_img)
            #     plt.pause(0.001)

        # Compute path
        current = goal
        self.path = [current]
        while current != start:
            current = self.came_from[current]
            self.path.append(current)
        self.path.append(start)
        self.path.reverse()

    def init_robot(self, robot_N):

        if robot_N==4:
            path = np.array([[16,-20],[16,47],[38,47]])
            kalpha = 1
            krho = 1
            pose = self.position
            for node in path:
                error = (node - pose[0:2])
                AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                while abs(AngleToGoal) > 0.5 and not rospy.is_shutdown():
                    error = (node - pose[0:2])
                    AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                    av_z = kalpha * AngleToGoal
                    self.update_velocity(0, 0, 0, 0, 0, av_z)
                    pose = self.position

                while (np.linalg.norm((node - pose[0:2])) > 0.5) and not rospy.is_shutdown():
                    error = (node - pose[0:2])
                    AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                    goalDist = np.linalg.norm(error)
                    v_x = krho * goalDist
                    av_z = kalpha * AngleToGoal
                    self.update_velocity(v_x, 0, 0, 0, 0, av_z)
                    pose = self.position

        elif robot_N==3:
            path = np.array([[-34,8],[-32,40]])
            kalpha = 1
            krho = 1

            t0 = rospy.Time.now().to_sec()
            t1 = t0
            while (t1 - t0) < 5:
                self.update_velocity(0, 0, 0, 0, 0, 0)
                t1 = rospy.Time.now().to_sec()
            while (t1-t0)<2:
                self.update_velocity(1, 0, 0, 0, 0, 0)
                t1 = rospy.Time.now().to_sec()

            pose = self.position
            for node in path:
                error = (node - pose[0:2])
                AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                while abs(AngleToGoal) > 0.5 and not rospy.is_shutdown():
                    error = (node - pose[0:2])
                    AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                    av_z = kalpha * AngleToGoal
                    self.update_velocity(0, 0, 0, 0, 0, av_z)
                    pose = self.position

                while (np.linalg.norm((node - pose[0:2])) > 0.5) and not rospy.is_shutdown():
                    error = (node - pose[0:2])
                    AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                    goalDist = np.linalg.norm(error)
                    v_x = krho * goalDist
                    av_z = kalpha * AngleToGoal
                    self.update_velocity(v_x, 0, 0, 0, 0, av_z)
                    pose = self.position

        elif robot_N == 2:
            path = np.array([[-36, -15], [-36, 16]])
            kalpha = 1
            krho = 1

            t0 = rospy.Time.now().to_sec()
            t1 = t0
            while (t1 - t0) < 10:
                self.update_velocity(0, 0, 0, 0, 0, 0)
                t1 = rospy.Time.now().to_sec()
            while (t1 - t0) < 2:
                self.update_velocity(1, 0, 0, 0, 0, 0)
                t1 = rospy.Time.now().to_sec()

            pose = self.position
            for node in path:
                error = (node - pose[0:2])
                AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                while abs(AngleToGoal) > 0.5 and not rospy.is_shutdown():
                    error = (node - pose[0:2])
                    AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                    av_z = kalpha * AngleToGoal
                    self.update_velocity(0, 0, 0, 0, 0, av_z)
                    pose = self.position

                while (np.linalg.norm((node - pose[0:2])) > 0.5) and not rospy.is_shutdown():
                    error = (node - pose[0:2])
                    AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                    goalDist = np.linalg.norm(error)
                    v_x = krho * goalDist
                    av_z = kalpha * AngleToGoal
                    self.update_velocity(v_x, 0, 0, 0, 0, av_z)
                    pose = self.position

        elif robot_N == 1:
            t0 = rospy.Time.now().to_sec()
            t1 = t0
            while (t1 - t0) < 60:
                self.update_velocity(0, 0, 0, 0, 0, 0)
                t1 = rospy.Time.now().to_sec()


    def is_in_obstacle(self):
        pos = self.transform_to_pixel(self.position[0:2])
        neighbor = self.position[0:2]
        while self.map[pos[1],pos[0]]<125:
            
            start_row = pos[1] - 40 if pos[1] - 40 > 0 else 0
            end_row = pos[1] + 40 if pos[1] + 40 < self.columns else self.columns
            start_colum = pos[0] - 40 if pos[0] - 40 > 0 else 0
            end_colum = pos[0] + 40 if pos[0] + 40 < self.rows else self.rows

            neighbor = self.map[start_row:end_row, start_colum:end_colum]
            index = np.argwhere(neighbor > 0)
            neighbor = [index[random.randint(0, len(index) - 1)][0] + start_row,
                    index[random.randint(0, len(index) - 1)][1] + start_colum]
            neighbor[0], neighbor[1] = neighbor[1], neighbor[0]
            neighbor = self.trasform_to_real(neighbor)

            kalpha = 0.5
            krho = 0.5
            pose = self.position
            error = (neighbor - pose[0:2])
            AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
            while abs(AngleToGoal) > 0.5 and not rospy.is_shutdown():
                error = (neighbor - pose[0:2])
                AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                av_z = kalpha * AngleToGoal
                self.update_velocity(0, 0, 0, 0, 0, av_z)
                pose = self.position

            while (np.linalg.norm((neighbor - pose[0:2])) > 0.5) and not rospy.is_shutdown():
                error = (neighbor - pose[0:2])
                AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                goalDist = np.linalg.norm(error)
                v_x = krho * goalDist
                av_z = kalpha * AngleToGoal
                self.update_velocity(v_x, 0, 0, 0, 0, av_z)
                pose = self.position
            pos = self.transform_to_pixel(self.position[0:2])

        return neighbor



    def findpath(self, start, goal):

        start = self.transform_to_pixel(start)
        goal = self.transform_to_pixel(goal)

        start_row = goal[1]-50 if goal[1]-50>0 else 0
        end_row = goal[1]+50 if goal[1]+50<self.columns else self.columns
        start_colum = goal[0]-50 if goal[0]-50>0 else 0
        end_colum = goal[0]+50 if goal[0]+50<self.rows else self.rows

        goal_neighbor = self.map[start_row:end_row, start_colum:end_colum]
        index = np.argwhere(goal_neighbor>0)
        goal = (index[random.randint(0,len(index)-1)][0]+start_row, index[random.randint(0,len(index)-1)][1]+start_colum)
        start[0], start[1] = start[1], start[0]
        start = (start[0], start[1])

        self.astar(start, goal)

        self.path = np.array(self.path)
        self.path[:, [0, 1]] = self.path[:, [1, 0]]
        path = self.trasform_to_real(self.path)
        l = int(self.d_subsample / self.scale)
        path_subsample = np.append(path[::l], [path[-1]], axis=0)
        result_img = self.draw_path(start, goal)

        # plt.imshow(result_img)
        # plt.show()
        return np.array(path_subsample)


    def move(self, robot_N):
        # Starts a new node
        assert int(robot_N) in {1, 2, 3, 4}

        # Receiveing the user's input
        krho = 2
        kalpha = 2
        # goal = np.array([[74.0,13],[10.9, -35.0],[0,1]])
        goal=self.goals
        while not rospy.is_shutdown():
            '''
            Robot plan a trajectory to arrive at goal
            '''
            # print("Init successful !! husky status : wait ->{}".format(self.wait))

            # verify the robot if vacant. If vacant, accept
            for sub_goal in goal:
                self.wait = True

                if self.wait == True:
                    print('Accept goal : {}'.format(sub_goal))
                    self.wait = False
                    print("control husky {}".format(robot_N), "husky status : wait ->{}".format(self.wait))
                    start = self.is_in_obstacle()

                    path = self.findpath(start, sub_goal)
                    print('Path Already, GO')

                    for node in path:
                        ## The robot plan a trajectory #

                        # turn to direction of target ##
                        pose = self.position
                        error = (node - pose[0:2])
                        AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                        while abs(AngleToGoal) > 1 and not rospy.is_shutdown():
                            error = (node - pose[0:2])
                            AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                            av_z = kalpha * AngleToGoal
                            self.update_velocity(0, 0, 0, 0, 0, av_z)
                            pose = self.position

                        # Move along the path
                        while (np.linalg.norm((node - pose[0:2])) > 1) and not rospy.is_shutdown():
                            error = (node - pose[0:2])
                            if self.close == 0:
                                #print('Collision')
                                t0 = rospy.Time.now().to_sec()
                                t1 = t0
                                while (t1 - t0) < 1.5:
                                    v_x = -1
                                    av_z = 0
                                    self.update_velocity(v_x, 0, 0, 0, 0, av_z)
                                    t1 = rospy.Time.now().to_sec()

                            # collision = self.will_collision(path)
                            if self.collison:
                                #print('Will collision')
                                AngleToGoal = 0.5 * self.left
                                goalDist = 1
                                v_x = krho * goalDist
                                av_z = kalpha * AngleToGoal
                                t0 = rospy.Time.now().to_sec()
                                t1 = t0
                                while (t1 - t0) < 1:
                                    self.update_velocity(v_x, 0, 0, 0, 0, av_z)
                                    t1 = rospy.Time.now().to_sec()
                                continue
                            else:
                                AngleToGoal = angle_wrap(math.atan2(error[1], error[0]) - pose[3])
                                goalDist = np.linalg.norm(error)
                                v_x = krho * goalDist
                                av_z = kalpha * AngleToGoal
                                self.update_velocity(v_x, 0, 0, 0, 0, av_z)
                                pose = self.position
                print("[robot] robot {} arrives at person".format(robot_N))
                self.goals.remove(self.goals[0])

if __name__ == '__main__':
    # Testing our function
    rospy.init_node('move_husky', anonymous=True)

    robot_N = rospy.get_param('~robot_N')
    path = pathfind(robot_N)
    print('husky{}---Finish Initialization ---'.format(robot_N))

    # path.init_robot(robot_N)
    path.move(robot_N)
