#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Library for robot control

Author: Adalberto Oliveira
Project: Solix AgBot
Version: 1.0
Date: 7-7-2022

"""
import rospy, time, angles, copy, math, sys, os,serial, geopy.distance
import numpy as np
from math import cos, sin, tan, atan2
from nav_msgs.msg import Odometry
from robot_control.msg import Speed, MotorStatus
from geometry_msgs.msg import Pose2D,Twist

class RobotControl:

    def __init__(self,communication=None,phisical=None):

        """
            Initializing the control class.

        """
        # Communication parameters
        if communication is not None:
            self.port =communication[0]
            self.baud_rate = communication[1]
            self.timeout =communication[2]
            self.motor = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)

        # Robot parameters
        if phisical is not None:
            self.separation = phisical[0]
            self.left_wheel = phisical[1]
            self.right_wheel = phisical[2]
            self.L = phisical[0]/2
            self.MAX_SPEED = phisical[3]
            self.MOTOR_GAIN_L = phisical[4]
            self.MOTOR_GAIN_R = phisical[5]
       

    def show(self):

        print(f"*** Robot Parameters:\
              \n* Separation: {self.separation}\
              \n* Left Wheel: {self.left_wheel}\
              \n* Right Wheel: {self.left_wheel}")


    def get_speed(self,cmd_vel):
        
        # Getting the 
        v = cmd_vel.linear.x
        omega = cmd_vel.angular.z

        phi_l = ((v - (self.L*omega))/self.left_wheel)
        phi_r = ((v + (self.L*omega))/self.right_wheel)

        phi_l = ((v - (self.L*omega))/self.left_wheel)*self.MOTOR_GAIN_L
        phi_r = ((v + (self.L*omega))/self.right_wheel)*self.MOTOR_GAIN_R


        # Speed limitation
        if abs(phi_l) > self.MAX_SPEED:
            phi_l = np.sign(phi_l)*self.MAX_SPEED

        if abs(phi_r) > self.MAX_SPEED:
            phi_r = np.sign(phi_r)*self.MAX_SPEED
        
        phi_l, phi_r = int(phi_l), int(phi_r)
        
        return phi_l, phi_r
    def get_speed(self,cmd_vel):

        # Getting the
        v = cmd_vel.linear.x
        omega = cmd_vel.angular.z

        phi_l = ((v - (self.L*omega))/self.left_wheel)
        phi_r = ((v + (self.L*omega))/self.right_wheel)

        phi_l = ((v - (self.L*omega))/self.left_wheel)*self.MOTOR_GAIN_L
        phi_r = ((v + (self.L*omega))/self.right_wheel)*self.MOTOR_GAIN_R


        # Speed limitation
        if abs(phi_l) > self.MAX_SPEED:
            phi_l = np.sign(phi_l)*self.MAX_SPEED

        if abs(phi_r) > self.MAX_SPEED:
            phi_r = np.sign(phi_r)*self.MAX_SPEED

        phi_l, phi_r = int(phi_l), int(phi_r)

        return phi_l, phi_r


    def get_cs(self,data,full=False):

        # Creating the Checksum
        checksum = 0
        for i in range(len(data)):
            checksum = checksum ^ ord(data[i])
        CS = "%02X" % checksum

        if full:
            return data+"*"+CS

        return CS
        

    def verify_cs(self,data):

        """
            Receives raw data and check the CS value
        """
        
        splited_data = data.split(",")
        CS = data.split(",")[-1]
        for i in range(len(CS)):
            if CS[i] == "*":
                CS = CS[i+1:i+3]
                splited_data[0]=splited_data[0][1:]
                splited_data[-1] = splited_data[-1][0:i]
                break
        
        data = ','.join(map(str,splited_data))
        checksum = self.get_cs(data)
        
        if CS == checksum:
            return True

        return False


    def run(self,cmd_vel):

        """
            Control method using robot parameters.

        """

        phi_l, phi_r = self.get_speed(cmd_vel)

        # Converting the data
        info = f"CMD,{phi_l},{phi_r}"
        CS = self.get_cs(info)

        # Packing the sending data
        data = "$"+info+"*"+str(CS)+"\n"
        self.motor.write(data.encode())

        # Reading the serial port
        received = self.motor.readline()
        received = received.decode('utf-8')

        print(f"Received data from serial:{received}\
                \nReceived CS: {self.verify_cs(received)}")

        return phi_l, phi_r
        

    def set_speed(self,phi_l,phi_r):

        """
            Control method using robot parameters.

        """

        # Converting the data
        info = f"CMD,{phi_l},{phi_r}"
        CS = self.get_cs(info)

        # Packing the sending data
        data = "$"+info+"*"+str(CS)+"\n"
    
        # Writing at the serial port
        self.motor.write(data.encode())

        received = None

        if False:
            # Reading the serial port
            received = self.motor.readline()
            received = received.decode('utf-8')
            print(f"Received data from serial:{received} \
                    \nReceived data size: {len(received)} \
                    \nReceived CS: {self.verify_cs(received)}")

        motor_status_data = self.get_motor_status(received=received)


        return motor_status_data
        


    def get_motor_status(self,received=None):

        motor_status_data = MotorStatus()
        motor_status_data.id_frame = "Speed control"

        if received is not None:
            splited_data = received.split(",")

            motor_status_data.left_speed = splited_data[0]
            motor_status_data.right_speed = splited_data[1]
            motor_status_data.left_alarm = splited_data[2]
            motor_status_data.right_alarm = splited_data[3]
            motor_status_data.remote_controlled = splited_data[4]

        return motor_status_data

class LineControl:

    def __init__(self):

        self.origin = Pose2D()
        self.line = [0,0,0]
        self.start = [0,0]
        self.stop = [0,0]
        self.curr_pose = Pose2D()
        self.int_rho = 0
        self.h = 0.01

    def set_origin(self,curr_pose):

        """
            Receives a point with lat/lon and defines as robot origin
            to be used in positioning computation.
        """

        self.origin = curr_pose

    def get_local_pose(self, gps_pose,l = 0.5,is_point=False):

        """
            Receives global points (GPS lat/lon coordinates and heading),
            and return a local point, using the robot start point as origin.
        """
        local_pose = Pose2D()

        if is_point:
            d =  geopy.distance.geodesic((self.origin.x,self.origin.y), gps_pose).m

            # getting the point (x,y)
            #gps_point = (gps_pose.x,gps_pose.y)
            origin_point = (self.origin.x,self.origin.y)
            ang = np.array(gps_pose) - np.array(origin_point)
            ang = math.atan2(ang[1],ang[0]) - self.origin.theta
            x,y = d*math.cos(ang),d*math.sin(ang)
            #thetha = gps_pose.theta - self.origin.theta
            local_pose.x = x - l*math.cos(ang)
            local_pose.y = y - l*math.sin(ang)
            #local_pose.theta = theta

        else:
            # Getting the geodesic distance from origin to point
            gps_point = (gps_pose.x,gps_pose.y)
            origin_point = (self.origin.x,self.origin.y)
            d =  geopy.distance.geodesic(origin_point, gps_point).m
            
            # getting the point (x,y)
            ang = np.array(gps_point) - np.array(origin_point)
            ang = math.atan2(ang[1],ang[0]) - self.origin.theta
            x,y = d*math.cos(ang),d*math.sin(ang)
            theta = gps_pose.theta - self.origin.theta

            local_pose.x = x - l*math.cos(theta)
            local_pose.y = y - l*math.sin(theta)
            local_pose.theta = theta # gps_pose.theta - self.origin.theta
        

        return local_pose

    def reset_integral(self):
        self.int_rho = 0


    def get_global_point(self,point):

        """
            Receives local point, using the robot start point as origin 
            and return a global GPS point (lat, lon).
        """

        lat,lon = 0,0

        gps_point = [lat,lon]


        return gps_point

    
    def set_points(sefl,start=None,stop=None):

        if start is not None:
            self.start = start
        
        if stop is not None:
            self.stop = stop

    
    def set_line(self,mission,local=False):

        if local:
            # Recovering the local mission points
            self.start = [mission.start_latitude,mission.start_longitude]
            self.stop = [mission.stop_latitude,mission.stop_longitude]

        else:
            # Recovering the global mission points
            start = [mission.start_latitude,mission.start_latitude]
            stop = [mission.stop_latitude,mission.stop_latitude]

            # Converting from global to local points
            self.start = self.get_local_pose(start,is_point=True)
            self.stop = self.get_local_pose(stop,is_point=True)

        # Computing the line
        a = self.start[1] - self.stop[1]
        b = self.stop[0] - self.start[0]
        c = self.start[0]*self.stop[1] - self.stop[0]*self.start[1]

        self.line = [a,b,c]

        return self.line

 
    def get_point_distance(self,curr_pose,start=True,stop=True,local=False):
        
        if local:

            # Distance to start point
            dx = self.start[0] - curr_pose.x
            dy = self.start[1] - curr_pose.y
            start_dist = np.sqrt(dx**2+dy**2)

            # Distance to stop point
            dx = self.stop[0] - curr_pose.x
            dy = self.stop[1] - curr_pose.y
            stop_dist = np.sqrt(dx**2+dy**2)

            if start and not stop:
                return start_dist
            elif stop and not start:
                return stop_dist
            else:
                return start_dist,stop_dist

        else:
            # Using lat/lon coordinates
            point = (curr_pose.x,curr_pose.y)
            start_dist = geopy.distance.geodesic(self.start, point).m
            stop_dist = geopy.distance.geodesic(self.stop, point).m

            if start and not stop:
                return start_dist
            elif stop and not start:
                return stop_dist
            else:
                return start_dist,stop_dist


    def get_control(self,v,curr_pose,gains,local_coord=False):

        cmd_vel = Twist()

        if local_coord:
            # Recovering local coordinate
            x = curr_pose.x
            y = curr_pose.y
            theta = curr_pose.theta

        else:
            # Converting from GPS to local coordinate
            local_pose = self.get_local_pose(curr_pose)
            x = local_pose.x
            y = local_pose.y
            theta = local_pose.theta

        print(f"Robot current pose:\n  x: {x}\n  y: {y}\n  th: {theta}")
        # Setting the gains
        Kd = gains[0]
        Kh = gains[1]

        # Normal distance to the line
        d = (self.line[0]*x + self.line[1]*y + self.line[2])/(np.sqrt(self.line[0]**2 + self.line[1]**2))
        alpha_d = -Kd*d


        # Robot orientation related to the line
        theta_l = math.atan2(-self.line[0],self.line[1])
        alpha_h = Kh*angles.shortest_angular_distance(theta,theta_l)

        # Control signal
        omega = round(alpha_d+alpha_h, 3)
        v = v - v*(abs(np.tanh(omega)))

        # Features:
        print(f"Line: {self.line}\nd: {d}\nalpha_h: {alpha_h/Kh}")


        cmd_vel.linear.x = v
        cmd_vel.angular.z = omega

        return cmd_vel


    def go_to_start(self,curr_pose,gains,local_coord=False):

        # Recovering the gains
        Kr = gains[0]
        Ka = gains[1]
        Kb = gains[2]

        if local_coord:
            # Recovering local coordinate
            local_pose = curr_pose

        else:
            # Converting from GPS to local coordinate
            local_pose = self.get_local_pose(curr_pose)
  

        local_start = self.get_local_pose(self.start,is_point=True) 
        local_stop = self.get_local_pose(self.stop,is_point=True) 

        # Converting to polar coordinates
        dx = local_start.x - curr_pose.x
        dy = local_start.y - curr_pose.y
        heading = math.atan2(dy,dx)

        # Getting the desired theta
        dx_d = local_stop.x - local_start.x
        dy_d = local_stop.y - local_start.y
        theta_d = math.atan2(dy_d,dx_d)

        # Working in local coordinates
        dx = local_start.x - local_pose.x
        dy = local_start.y - local_pose.y

        # Creating the polar features 
        rho = round(np.sqrt(dx**2+dy**2),2)      
        alpha = round(angles.shortest_angular_distance(local_pose.theta,heading),3)
        beta = round(angles.shortest_angular_distance(theta_d,(local_pose.theta+alpha)),3) 
        ang_err = round(angles.shortest_angular_distance(theta_d,local_pose.theta),3)

        if True:
            print(f"Polar Features:\
                    \n Theta: {local_pose.theta}\
                    \n Heading: {heading}\
                    \n rho: {rho}\
                    \n alpha: {alpha}\
                    \n beta: {beta}\
                    \n theta_d: {theta_d}\
                    \n angular error: {ang_err}")
        
        #Linear control law
        #v = Kr*rho
        #omega = Ka*alpha + Kb*beta 

        # Non-linear control law
        v = Kr*rho*np.cos(alpha)
        omega = Ka*alpha + (Ka*np.sin(alpha)*np.cos(alpha)*(alpha+Kr*beta))/alpha

        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = omega

        return cmd_vel,ang_err


    def go_to_stop(self,v,curr_pose,w=1,line_gains=[1,1],polar_gains=[1,1,1],local_coord=False):

        cmd_vel = Twist()

        if local_coord:
            # Recovering local coordinate
            x = curr_pose.x
            y = curr_pose.y
            theta = curr_pose.theta

        else:
            # Converting from GPS to local coordinate
            local_pose = self.get_local_pose(curr_pose)
            x = local_pose.x
            y = local_pose.y
            theta = local_pose.theta
            local_stop = Pose2D()
            local_stop.x = self.stop[0]
            local_stop.y = self.stop[1]
            local_stop = self.get_local_pose(local_stop)

        print(f"Robot current pose:\n  x: {x}\n  y: {y}\n  th: {theta}")
        
        # Setting the gains
        Kd = line_gains[0]
        Kh = line_gains[1]
        Kr = polar_gains[0]
        Ka = polar_gains[1]
        Kb = polar_gains[2]

 
        # Normal distance to the line
        d = (self.line[0]*x + self.line[1]*y + self.line[2])/(np.sqrt(self.line[0]**2 + self.line[1]**2))
        alpha_d = -Kd*d


        # Robot orientation related to the line
        theta_l = math.atan2(-self.line[0],self.line[1])
        alpha_h = Kh*angles.shortest_angular_distance(theta,theta_l)
        
        omega_line = round(alpha_d+alpha_h, 3) 
        v_line = v - v*(abs(np.tanh(omega_line)))

        # Converting to polar coordinates
        dx = self.stop[0] - curr_pose.x
        dy = self.stop[1] - curr_pose.y
        heading = math.atan2(dy,dx)
        
        x_line = self.stop[0] - self.start[0]
        y_line = self.stop[1] - self.start[1]
        theta_d = math.atan2(y_line,x_line)
                

        # Working in local coordinates
        dx = local_stop.x - x
        dy = local_stop.y - y

        # Creating the polar features 
        rho = round(np.sqrt(dx**2+dy**2),2)      
        alpha = round(angles.shortest_angular_distance(theta,heading),3)
        beta = round(angles.shortest_angular_distance(theta_d,(theta+alpha)),3) 
        ang_err = round(angles.shortest_angular_distance(theta_d,theta),3)

        # Non-linear control law
        v_polar = Kr*rho*np.cos(alpha)
        #v_polar = Kr*rho

        omega_polar = Ka*alpha + (Kr*np.sin(alpha)*np.cos(alpha)*(alpha+Kb*beta))/alpha

        # Control signal
        v = v_polar
        omega = omega_line*w + (1-w)*omega_polar 


        # Features:
        print(f"Line: {self.line}\nd: {d}\nalpha_h: {alpha_h/Kh}")


        cmd_vel.linear.x = v
        cmd_vel.angular.z = omega

        return cmd_vel,ang_err

    
    def go_to_pose(self,curr_pose,destination="START",v_cruise=0,w=0,line_gains=[0,0],polar_gains=[0,0,0],local_coord=False):

        # Recovering the gains
        # Setting the gains
        Kd = line_gains[0]
        Kh = line_gains[1]
        Kr = polar_gains[0]
        Ka = polar_gains[1]
        Kb = polar_gains[2]
        Ki = Kd


        # Converting from GPS to local coordinate
        print("\nFrom GPS to local coordinate")
        local_curr_pose = self.get_local_pose(curr_pose)
  
        # Setting the local coordinate for start and stop points
        local_start = self.get_local_pose(self.start,is_point=True)
        local_stop = self.get_local_pose(self.stop,is_point=True)
        local_destination = Pose2D()

        # print(f"Local Start: \n{local_start}\nLocal Stop: \n{local_stop}")
        # print(f"\nOrigin pose:\n{self.origin}\nRobot Current pose: \n Local frame:\n{local_curr_pose}\n GPS frame:\n{curr_pose}")
        if destination.upper() == "START":
            local_destination = local_start
        elif destination.upper() == "STOP":
            local_destination = local_stop
        
        print(f"\nLocal Destination: {local_destination}")
        # else:
        #     local_destination = curr_pose


        # Computing the features for line regulation controlle
        theta_l = math.atan2(-self.line[0],self.line[1])
        line_distance = (self.line[0]*local_curr_pose.x + self.line[1]*local_curr_pose.y + self.line[2])/(np.sqrt(self.line[0]**2 + self.line[1]**2))
        alpha_h = Kh*angles.shortest_angular_distance(local_curr_pose.theta,theta_l)
        alpha_d = -Kd*line_distance

        # Computing the control signals for line regulation
        omega_line = round(alpha_d+alpha_h, 3) 
        v_line = v_cruise - v_cruise*(abs(np.tanh(omega_line)))

        # Converting to polar coordinates
        dx = local_destination.x - local_curr_pose.x
        dy = local_destination.y - local_curr_pose.y
        heading = math.atan2(dy,dx)

        # Getting the desired theta for polar regulation
        x_line = self.stop[0] - self.start[0]
        y_line = self.stop[1] - self.start[1]
        theta_d_g = math.atan2(y_line,x_line)

        dx_d = local_stop.x - local_start.x
        dy_d = local_stop.y - local_start.y
        theta_d = math.atan2(dy_d,dx_d)


        # Creating the polar features 
        rho = round(np.sqrt(dx**2+dy**2),2)
        alpha = round(angles.shortest_angular_distance(heading,local_curr_pose.theta),4)
        #beta = round(angles.shortest_angular_distance(local_curr_pose.theta,theta_d),3) 
        #beta = round(angles.shortest_angular_distance(theta_d,heading),3)
        #beta = heading + theta_d
        ang_err = round(angles.shortest_angular_distance(theta_d,local_curr_pose.theta),3)
        beta = ang_err
        print(f"*** Polar features:\n   rho: {rho}\n   A: {alpha}\n   B: {beta}")

        # Computing the integral therm
        self.int_rho = self.int_rho + self.h*rho

        # print(f"Integral therm: {self.int_rho}\nIntegral gain: {Ki}\nControl Signal: {self.int_rho*Ki}")

        #print(f"\nPolar values: rho: {rho}, alpha: {alpha}, beta: {beta}, angErr: {ang_err}")
        #print(f"\nLine values:\nLine Distance: {line_distance}\nAlpha H: {alpha_h}, Alpha D: {alpha_d}")
        # Non-linear control law
        v_polar = Kr*rho*np.cos(alpha) + self.int_rho*Ki
        #v_polar = Kr*rho
        Ka = 0 if rho < 0.5 else Ka
        Kb = 0 if (rho > 0.5) else Kb
        omega_polar = Ka*alpha + Kb*beta

        #omega_polar = Ka*alpha + Kr*((np.sin(alpha)*np.cos(alpha))/alpha)*(alpha+Kb*beta)
        omega_polar = 0 if math.isnan(omega_polar) else omega_polar

        # Control signal
        cmd_vel = Twist()
        cmd_vel.linear.x = v_polar
        cmd_vel.angular.z = (1-w)*omega_polar + omega_line*w

        return cmd_vel, ang_err, rho
