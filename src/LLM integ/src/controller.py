import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray, String
import math
from util import euler_to_quaternion, quaternion_to_euler
import time
from waypoint_list import WayPoints
import csv
import pandas as pd
import matplotlib.pyplot as plt
import json


class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = True
        self.accelist = []
        self.trajectoryx =[]
        self.trajectoryy =[]
        self.waypointsfollowed =[]

        self.straight_speed = 10
        self.corner_speed = 5

        rospy.Subscriber("/LLM_HVI", String, self.hvi_callback)

    def hvi_callback(self, data):

# Parse the JSON string
        data_json = json.loads(data.data)

        # Extract top_speed_straight and top_speed_corner
        top_speed_straight = data_json["top_speed_straight"]
        top_speed_corner = data_json["top_speed_corner"]

        print("Top Speed Straight changed to:", top_speed_straight)
        print("Top Speed Corner changed to:", top_speed_corner)

        self.straight_speed = top_speed_straight
        self.corner_speed = top_speed_corner


    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp


    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    def extract_vehicle_info(self, currentPose):

        ####################### TODO: Your TASK 1 code starts Here #######################
        pos_x, pos_y, vel, yaw = 0, 0, 0, 0

        msg = self.getModelState()
        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y
        vel = np.sqrt(msg.twist.linear.x**2 + msg.twist.linear.y**2)
        quaternion = msg.pose.orientation
        euler = quaternion_to_euler(quaternion.x,quaternion.y, quaternion.z, quaternion.w)
        yaw = euler[2]
        

        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################
      
            t1_x, t1_y = future_unreached_waypoints[0]
           
            if (abs(t1_x-curr_x)<0.25) or ((abs(t1_y-curr_y)<0.25)):
               
                target_velocity =self.straight_speed
                # print("straight")
            else:
                target_velocity = self.corner_speed  #8
            #     print("curve")   
            # print("future unreached waypoints", future_unreached_waypoints[0])
            #print("curernt xy", curr_x, curr_y)

            ####################### TODO: Your TASK 2 code ends Here #######################
            return target_velocity
    

    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

        ####################### TODO: Your TASK 3 code starts Here #######################
     
            p1 = future_unreached_waypoints[0]
            if future_unreached_waypoints[0] == [-16,-98]:
                target_point = future_unreached_waypoints[0]
            else:
                p2 = future_unreached_waypoints[1]
                if p1[0]-p2[0]>2 or p1[1]-p2[1]>2:
                # print ("way point to farrrrr")
                    target_point = p1 
                else:
                    target_point = p2
            curr_point = np.array([curr_x, curr_y])
                
                     
            ld = math.sqrt((target_point[0]-curr_x)**2 + (target_point[1]-curr_y)**2)
            alpha = np.arctan2((target_point[1]-curr_point[1]),(target_point[0]- curr_point[0])) - curr_yaw
            delta = (np.arctan2((2*1.75*np.sin(alpha)),(ld)))
            if delta>2:
                delta =2
            if delta <-2:
                delta =-2
            self.waypointsfollowed.append(target_point)
            way = pd.DataFrame(self.waypointsfollowed)
            way.to_csv('way.csv')
            # print("target xy", target_point)
            print("current velocity:", self.prev_vel)
            target_steering = delta
            
            return target_steering
    
    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)

        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz
            # print(" acceleration", acceleration)
            self.accelist.append(acceleration)
            self.trajectoryx.append(curr_x)
            self.trajectoryy.append(curr_y)
            self.prev_vel = curr_vel
            



        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)


        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)