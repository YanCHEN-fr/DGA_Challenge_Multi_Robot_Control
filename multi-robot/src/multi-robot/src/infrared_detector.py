#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates
from config import drone_angles
import numpy as np
import time
import os
import random
import time


class Block:
    def __init__(self, name):
        self._name = name


class Drone:
    def __init__(self, number, x, y, z):
        self.name = number
        self.x = x
        self.y = y
        self.z = z


class Person:
    def __init__(self, number, x, y, z):
        self.name = number
        self.x = x
        self.y = y
        self.z = z


class Visible:
    def __init__(self, person, drone):
        self.drone = drone
        self.person = person


class InfraRed:

    _peopleDict = {
        'block_a': Block('hostile_1'),
        'block_b': Block('hostile_2'),
        'block_c': Block('hostile_3'),
        'block_d': Block('hostile_4'),
        'block_e': Block('hostile_5'),
        'block_f': Block('hostile_6'),
        'block_g': Block('hostile_7'),
        'block_h': Block('hostile_8')
    }

    _dronesDict = {
        'block_a': Block('drone1'),
        'block_b': Block('drone2'),
        'block_c': Block('drone3'),
        'block_d': Block('drone4'),
        'block_e': Block('drone5'),
        'block_f': Block('drone6'),
        'block_g': Block('drone7'),
        'block_h': Block('drone8'),
        'block_i': Block('drone9'),
        'block_j': Block('drone10')
    }

    def __init__(self, drone_number):

        # Starts a new node
        self.drone_number = drone_number
        self.drones = []
        self.people = []
        self.people_visible_by_drones = []
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         callback=self.update_positions, queue_size=10)
        self.pos_pub=rospy.Publisher("/person/real/position", Point,queue_size=1)
        time.sleep(1)
        while not rospy.is_shutdown():
            try:
                self.looking_for_people()
            except KeyboardInterrupt:
                print('Interrupted')
                sys.exit(0)

    def get_infra_range(self, drone_height):
        width = 2.0 * drone_height * drone_angles["wfov"]/2.0
        height = 2.0 * drone_height * drone_angles["hfov"] / 2.0
        return width, height

    def looking_for_people(self):
        self.people_visible_by_drones = []

        if self.drone_number == 0:
            for drone in self.drones:
                for person in self.people:
                    width, lenght = self.get_infra_range(drone.z)
                    if (np.abs(drone.x - person.x) < width/2.0) and \
                            (np.abs(drone.y - person.y) < lenght / 2.0) and \
                            drone.z < 10.0:

                        self.people_visible_by_drone(person, drone)

        else:
            for drone in self.drones:
                for person in self.people:
                    width, lenght = self.get_infra_range(drone.z)
                    if (np.abs(drone.x - person.x) < width / 2.0) and \
                            (np.abs(drone.y - person.y) < lenght / 2.0) and \
                            drone.z < 10.0 and self.drone_number == drone.name:
                        self.people_visible_by_drone(person, drone)

        if self.people_visible_by_drones:
            for people_seen_by_drone in self.people_visible_by_drones:
                people_pos=Point()
                people_pos.x=people_seen_by_drone.person.x
                people_pos.y=people_seen_by_drone.person.y

                self.pos_pub.publish(people_pos)
                # print("x={},y={}".format(people_pos.x,people_pos.y))                
                # print("Hostile number {} is being seen by drone number {}".
                #       format(people_seen_by_drone.person.name, people_seen_by_drone.drone.name))

    def update_positions(self, data):
        self.get_people_positions(data)
        self.get_drone_positions(data)

    def people_visible_by_drone(self, person, drone):
        visible = Visible(person=person, drone=drone)
        self.people_visible_by_drones.append(visible)

    def get_drone_positions(self, data):
        self.drones = []
        for block in self._dronesDict.itervalues():
            blockName = str(block._name)
            if blockName in data.name:
                drone_index = data.name.index(blockName)
                current_drone = Drone(number=int(blockName[-1]),
                                      x=data.pose[drone_index].position.x,
                                      y=data.pose[drone_index].position.y,
                                      z=data.pose[drone_index].position.z)
                self.drones.append(current_drone)

                # print('\n')
                # print(blockName)
                # print("Position x: " + str(current_drone.x))
                # print("Position y: " + str(current_drone.y))
                # print("Position z: " + str(current_drone.z))

    def get_people_positions(self, data):
        self.people = []
        for block in self._peopleDict.itervalues():
            blockName = str(block._name)
            if blockName in data.name:
                person_index = data.name.index(blockName)
                current_person = Person(number=int(blockName[-1]),
                                        x=data.pose[person_index].position.x,
                                        y=data.pose[person_index].position.y,
                                        z=data.pose[person_index].position.z)

                self.people.append(current_person)

                # print('\n')
                # print(blockName)
                # print("Position x: " + str(current_people.x))
                # print("Position y: " + str(current_people.y))
                # print("Position z: " + str(current_people.z))


if __name__ == '__main__':
    rospy.init_node('infraRed_detect', anonymous=True)
    
    try:
        # print("Let's get the detections of a drone")
        # drone_N = input("Select a drone . (Options 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0 for all drones): ")
        drone_N=int(rospy.get_param("~drone_N"))
        k = InfraRed(drone_N)

    except rospy.ROSInterruptException:
        print('Interruption')
        pass
