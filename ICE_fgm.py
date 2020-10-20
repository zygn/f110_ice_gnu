#!/usr/bin/env python

import rospy
import math
import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class Pure_Pursuit:
    def __init__(self):
        
        self.ackermann_data = AckermannDriveStamped()

        self.scan_range_size = 0
        self.goal_gap = 0

        self.PI = 3.141592
        self.rf_distance = 2.5
        self.RACECAR_LENGTH = 0.325

        self.SPEED_MAX = rospy.get_param('max_speed', 8.0)
        self.SPEED_MIN = rospy.get_param('min_speed', 1.5)
        self.RATE = rospy.get_param('rate', 100)

        # configuration values for driving algorithm
        self.ROBOT_SCALE = rospy.get_param('robot_scale', 0.35)
        self.THRESHOLD = rospy.get_param('threshold', 5.0)
        self.GAP_SIZE = rospy.get_param('gap_size', 1)
        self.FILTER_SCALE = rospy.get_param('filter_scale', 1.1) 
        self.GAP_THETA_GAIN = rospy.get_param('gap_theta_gain', 20.0)
        self.REF_THETA_GAIN = rospy.get_param('ref_theta_gain', 1.5)
        self.speed_gain = 0
        self.steering_gain = 0
        self.gain_cont = 0
        self.speed_cont = 0
        self.speed_gain_op = 0
        self.a = 0
        self.b = 0
        self.c = 0
        self.speed_coef = (4*(self.SPEED_MAX-self.SPEED_MIN))/(pow(self.PI,2))
        self.ref_point_rt = [0,0]


        self.ackermann_data.drive.acceleration = 0
        self.ackermann_data.drive.jerk = 0
        self.ackermann_data.drive.steering_angle = 0
        self.ackermann_data.drive.steering_angle_velocity = 0

        self.rf_num = 1

        self.waypoints = self.get_waypoint()
        self.wp_index_current = 0
        self.current_position = [0]*3
        self.nearest_distance = 0

        self.max_angle = 0
        self.ref_angle = 0

        #GAP class
        self.gaps = []
        self.for_gap = [0, 0, 0]
        self.scan_angle_increment = 0.00435
        self.range_mid_idx = 0
        self.theta_for = self.PI/3
        self.gap_cont = 0

        self.start_idx_temp = 0
        self.end_idx_temp = 0
        self.max_temp = 0
        self.max_idx_temp = 0
        self.for_point = 0

        rospy.Subscriber('/scan', LaserScan, self.subCallback_scan, queue_size = 10)
        rospy.Subscriber('/odom', Odometry, self.Odome, queue_size = 10)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size = 10 )

    #ready to drive
    def get_waypoint(self):
        file_wps = np.genfromtxt('../catkin_ws/src/car_duri/wp_vegas.csv',delimiter=',',dtype='float')
        temp_waypoint = []
        for i in file_wps:
            wps_point = [i[0],i[1],0]
            temp_waypoint.append(wps_point)
            self.rf_num += 1
        print("rf",self.rf_num)
        return temp_waypoint

    def find_desired_wp(self):
        #print("3")
        wp_index_temp = self.wp_index_current
        self.nearest_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

        while True:
            wp_index_temp+=1

            if wp_index_temp >= self.rf_num-1:
                wp_index_temp = 0
            temp_distance = self.getDistance(self.waypoints[wp_index_temp], self.current_position)

            if temp_distance < self.nearest_distance:
                self.nearest_distance = temp_distance
                self.wp_index_current = wp_index_temp
            elif ((temp_distance > (self.nearest_distance + self.rf_distance*1.2)) or (wp_index_temp == self.wp_index_current)):
                break
        
        temp_distance = 0
        idx_temp = self.wp_index_current
        while True:
            if idx_temp >= self.rf_num-1:
                idx_temp = 0
            temp_distance = self.getDistance(self.waypoints[idx_temp], self.current_position)
            if temp_distance > self.rf_distance: break
            idx_temp += 1

        transformed_nearest_point = self.transformPoint(self.current_position, self.waypoints[idx_temp])

        self.ref_point_rt = self.xyt2rt(transformed_nearest_point)


        # self.ref_point_rt[1] = self.ref_point_rt[1] * 1.3

    def drive_with_wp(self, goal):
        #print("7")
        #steering angle calculation
        #goal - [2] = max_idx, 
        self.max_angle = (goal[2] - self.range_mid_idx)*self.scan_angle_increment
        self.ref_angle = self.ref_point_rt[1]

        #step = (int)(self.ref_angle/self.scan_angle_increment)
        #ref_idx = self.range_mid_idx + step

        #range_min_values = [0]*10
        temp_avg = 0
        dmin = 0
        for i in range(10):
            
            dmin += self.scan_filtered[i]
        dmin /= 10
   
        i = 0
        while i < self.scan_range_size-7:
            j = 0
            while j < 10:
                if i + j > 1079:
                    temp_avg += 0
                else:
                    temp_avg += self.scan_filtered[i+j]
                j += 1
                
            temp_avg /= 10
            
            if dmin > temp_avg:
                if temp_avg == 0:
                    temp_avg = dmin
                dmin = temp_avg
            temp_avg = 0
            i += 3

        if dmin == 0:
            dmin = 1
            
        controlled_angle = ( (self.GAP_THETA_GAIN/dmin)*self.max_angle + self.REF_THETA_GAIN*self.ref_angle)/(self. GAP_THETA_GAIN/dmin + self.REF_THETA_GAIN) 
        #print("con",controlled_angle)
        distance = 1.5
        path_radius = distance / (2*np.sin(controlled_angle))
        steering_angle = np.arctan(self.RACECAR_LENGTH/path_radius)
        if steering_angle < 0:
            self.steering_gain = -self.steering_gain 

        if (np.fabs(steering_angle) > self.PI/8):
            speed = self.SPEED_MIN
            self.speed_gain = 0
            #print("steering_angle",steering_angle)
            #self.steering_gain = 10*self.steering_gain
        else:
            speed = (float)(-(3/self.PI)*(self.SPEED_MAX-self.SPEED_MIN)*np.fabs(self.max_angle)+self.SPEED_MAX)
            speed = np.fabs(speed)

        if speed < self.speed_gain:
            speed = self.SPEED_MIN
            self.speed_gain = 0

        accel = speed - self.speed_gain
        if accel < self.SPEED_MIN:
            accel = self.SPEED_MIN
        

        if np.fabs(steering_angle) < np.fabs(self.steering_gain):
            self.steering_gain = 0

        
        steering = steering_angle + self.steering_gain

        # if self.scan_filtered[545] < self.THRESHOLD - 4:
        #     if steering < 0:
        #         steering = -0.5
        #     else:
        #         steering = 0.5

        # publish ackermann message    
        self.ackermann_data.drive.steering_angle = steering
        # if np.fabs(self.steering_gain) > 0.02:
        #     print("steering_gain",self.steering_gain)
        #     print("steering",self.ackermann_data.drive.steering_angle)
        self.ackermann_data.drive.steering_angle_velocity = 0
        self.ackermann_data.drive.speed = accel
        #print("speed",speed,"gain",self.speed_gain)
        # if speed - self.speed_gain < 2:
        #     print(speed - self.speed_gain)
        self.ackermann_data.drive.acceleration = 0
        self.ackermann_data.drive.jerk = 0

        self.drive_pub.publish(self.ackermann_data)
        self.speed_gain = 0
        self.steering_gain = 0
        self.gain_cont = 0
        self.speed_gain_op = 0
        #self.a = 0
    #finish drive function

    def subCallback_scan(self,msg_sub):
        #print("1")
        #self.scan_angel_min = msg_sub.angle_min
        self.scan_angle_min = msg_sub.angle_min
        self.scan_angle_max = msg_sub.angle_max
        self.scan_angle_increment = msg_sub.angle_increment
        #print(self.scan_angle_increment)
        self.scan_range_size = len(msg_sub.ranges)
        self.range_mid_idx = (int)(self.scan_range_size/2)
        

        # allocate the scan data array
        #scan_ori = [0]*self.scan_range_size
        self.scan_origin = [0]*self.scan_range_size
        self.scan_filtered = [0]*self.scan_range_size
        #speed_gain_op = 0
        for i in range(self.scan_range_size):
            
            self.scan_origin[i] = msg_sub.ranges[i]
            self.scan_filtered[i] = msg_sub.ranges[i]
            if i > 0:
                #Determin corner or obs
                if (self.scan_filtered[i-1]*1.65 < self.scan_filtered[i]): #and self.gain_cont == 0:
                    #self.a += 1
                    #print("gains")
                    #catching frontal obs
                    if (i >= self.range_mid_idx - 60) and (i <= self.range_mid_idx + 60):
                        #self.b += 1
                        #print("i",i)
                        if(self.scan_filtered[i] < (self.THRESHOLD+10)):
                            #print("speed_down",self.speed_cont,"front",self.scan_filtered[self.range_mid_idx])
                            self.steering_gain -= 0.001
                            #speed_gain_op += 3
                            self.speed_gain = 2.5 # - speed_gain_op
                            #self.speed_cont += 1
                        else:
                            self.steering_gain -= 0.05
                            self.speed_gain -= 0.1
                            #
                    else:
                        #self.c += 1
                        self.steering_gain += 0.003
                        self.speed_gain -= 0.25
                        #print(self.speed_gain)
        
        #         elif (i >= self.range_mid_idx -3) and (i <= self.range_mid_idx + 3):
        #             if self.scan_filtered[i] < self.THRESHOLD-1.5:
        #                 #print("no")
        #                 #self.a += 1
        #                 self.steering_gain -= 0.4
        #                 self.speed_gain += 0.8
        # if self.speed_gain > 5:
        #     self.speed_gain = 5

        #print("a",self.a,"b",self.b,"c",self.c)
                        
                    #self.gain_cont = 1
        # if self.a > 0:
        #     print("a",self.a)
            
            # if msg_sub.ranges[i] <= 0.1:
            #     print("Hit")

        # zero filtering
        for i in range(self.scan_range_size):           
            if self.scan_origin[i] == 0:
                cont = 0 
                sum = 0
                for j in range(1,21):
                    if i-j >= 0:
                        if self.scan_origin[i-j] != 0:
                            cont += 1
                            sum += self.scan_origin[i-j]
                    if i+j < self.scan_range_size:
                        if self.scan_origin[i+j] != 0:
                            cont += 1
                            sum += self.scan_origin[i+j]
                self.scan_origin[i] = sum/cont
                self.scan_filtered[i] = sum/cont
            
            #end zero filtering

        #filtering the scan data
        for i in range(self.scan_range_size - 1):
            #right to left
            if self.scan_origin[i]*self.FILTER_SCALE < self.scan_filtered[i+1]:
                unit_length = self.scan_origin[i]*self.scan_angle_increment
                filter_num = self.ROBOT_SCALE/unit_length
                
                j = 1
                while j < filter_num+1:
                    if i+j < self.scan_range_size:
                        if self.scan_filtered[i+j] > self.scan_origin[i]:
                            self.scan_filtered[i+j] = self.scan_origin[i]
                        else: break
                    else: break 
                    j += 1
        
            #left to right
            elif self.scan_filtered[i] > self.scan_origin[i+1]*self.FILTER_SCALE:
                unit_length = self.scan_origin[i+1]*self.scan_angle_increment
                filter_num = self.ROBOT_SCALE / unit_length

                j = 0
                while j < filter_num + 1:
                    if i-j > 0:
                        if self.scan_filtered[i-j] > self.scan_origin[i+1]:
                            self.scan_filtered[i-j] = self.scan_origin[i+1]
                        else: break
                    else: break
                    j += 1

        #publish filtered scan data
    #find current_position && waypoint
    def Odome(self, odom_msg):
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w

        siny_cosp = 2.0 * (qw*qz + qx*qy)
        cosy_cosp = 1.0-2.0*(qy*qy + qz*qz)

        current_position_theta = np.arctan2(siny_cosp, cosy_cosp)
        current_position_x = odom_msg.pose.pose.position.x
        current_position_y = odom_msg.pose.pose.position.y
        self.current_position = [current_position_x,current_position_y, current_position_theta]

        self.find_desired_wp()

    def GAP(self):
        if self.gap_cont <= 1 :
            #print("2")
            self.for_point = (self.theta_for/self.scan_angle_increment)
            #[0] = start_idx, [1] = end_idx
            self.for_gap[0] = (self.range_mid_idx) - self.for_point
            self.for_gap[1] = (self.range_mid_idx) + self.for_point

            self.gap_cont += 1

    def gap_finding(self, scan):
        #print("4")
        self.gaps = []

        i = 0
        while i < self.scan_range_size - self.GAP_SIZE:

            if scan[i] > self.THRESHOLD:
                start_idx_temp = i
                end_idx_temp = i
                max_temp = scan[i]
                max_idx_temp = i
                
                for j in range(1,self.GAP_SIZE):

                    if scan[i] > self.THRESHOLD:
                        if scan[i] > max_temp:
                            max_temp = scan[i]
                            max_idx_temp = i
                        else:
                            max_temp = -1
                            break
                    #end checking is gap or not
                    #is not gap
                if max_temp == -1: break

                #is gap
                else:
                    while ((scan[i] > self.THRESHOLD) and (i+1 < self.scan_range_size )):
                        i += 1
                        if scan[i] > max_temp:
                            max_temp = scan[i]
                            max_idx_temp = i
                    i += 1
                    end_idx_temp = i

                    gap_temp = [0]*3
                    gap_temp[0] = start_idx_temp
                    gap_temp[1] = end_idx_temp
                    gap_temp[2] = max_idx_temp
                    self.gaps.append(gap_temp)
            i += 1
            #end gap finding loop
        #end gap finding method

    def for_gap_finding(self,scan):
        #print("5")
        self.for_point = (int)(self.theta_for/self.scan_angle_increment)
        #[0] = start_idx, [1] = end_idx
        start_idx_temp = (self.range_mid_idx) - self.for_point
        end_idx_temp = (self.range_mid_idx) + self.for_point

        max_idx_temp = start_idx_temp
        max_temp = scan[start_idx_temp]

        for i in range(start_idx_temp, end_idx_temp):
            if max_temp < scan[i]:
                max_temp = scan[i]
                max_idx_temp = i
        #[0] = start_idx, [1] = end_idx, [2] = max_idx_temp
        self.for_gap[0] = start_idx_temp
        self.for_gap[1] = end_idx_temp
        self.for_gap[2] = max_idx_temp


    def getDistance(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        
        return np.sqrt(dx**2 + dy**2)

    def xyt2rt(self, origin):
        rtpoint = []

        x = origin[0]
        y = origin[1]

        #rtpoint[0] = r, [1] = theta
        rtpoint.append(np.sqrt(x*x + y*y))
        rtpoint.append(np.arctan2(y, x) - (self.PI/2))

        return rtpoint
    
    def transformPoint(self, origin, target):
        theta = self.PI/2 - origin[2]

        dx = target[0] - origin[0]
        dy = target[1] - origin[1]
        dtheta = target[2] + theta
        
        tf_point_x = dx * np.cos(theta) - dy * np.sin(theta)
        tf_point_y = dx * np.sin(theta) + dy * np.cos(theta)
        tf_point_theta = dtheta
        tf_point = [tf_point_x, tf_point_y, tf_point_theta]
        
        return tf_point

    #get nearest gap from reference point
    #ref - [0] = r, [1] = theta
    def get_nearest_gap(self, ref):
        #print("6")
        num = len(self.gaps)
        if num == 0:
            return self.for_gap
        else:
            step = (int)(ref[1]/self.scan_angle_increment)
            ref_idx = self.range_mid_idx + step

            gap_idx = 0

            if self.gaps[0][0] > ref_idx:
                distance = self.gaps[0][0] - ref_idx
            elif self.gaps[0][1] < ref_idx:
                distance = ref_idx - self.gaps[0][1]
            else:
                distance = 0
                gap_idx = 0

            i = 1
            while (i < num):
                if self.gaps[i][0] > ref_idx:
                    temp_distance = self.gaps[i][0] - ref_idx
                    if temp_distance < distance:
                        distance = temp_distance
                        gap_idx = i
                elif self.gaps[i][1] < ref_idx:
                    temp_distance = ref_idx - self.gaps[i][1]
                    if temp_distance < distance:
                        distance = temp_distance
                        gap_idx = i
                    
                else:
                    temp_distance = 0
                    distance = 0
                    gap_idx = i
                    break

                i += 1
            
            return self.gaps[gap_idx]

        #end get nearest gap

    def driving(self):
        #RATE = loop_late(100)
        
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():

            if self.scan_range_size == 0: continue
            
            self.GAP()
            #self.find_desired_wp()
            self.gap_finding(self.scan_filtered)
            self.for_gap_finding(self.scan_filtered)

            self.goal_gap = self.get_nearest_gap(self.ref_point_rt)

            self.drive_with_wp(self.goal_gap)

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("test")
    A = Pure_Pursuit()
    A.driving()
    rospy.spin()
    
