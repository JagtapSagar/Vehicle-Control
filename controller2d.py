#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('v_error_previous', 0.0)
        self.vars.create_var('v_integral_error_previous', 0.0)


        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.

            sample_time = t - self.vars.t_previous

            #x_error = waypoints[][0]-x
            #y_error = waypoints[][1]-y

            # ==================================
            #  Implement kinematic model here
            # ==================================

            # throttle_output = 0
            # brake_output    = 0

            # Upper/Higher Level Controller
            # =============================
            # Calculates desired acceleration from velocity error
            # x''des = kp(x'ref-x') +ki*integral{(x'ref-x')dt} +kd*derivative{(x'ref-x')/dt}

            # PID Gains
            kp=0.5
            ki=0.3
            kd=0.13

            # Error calculation
            v_error = v_desired-v
            v_integral_error = self.vars.v_integral_error_previous + v_error*sample_time
            v_derivative_error = (v_error - self.vars.v_error_previous)/sample_time

            # PID (O/P here is desired acceleration)
            a = kp*v_error + ki*v_integral_error + kd*v_derivative_error

            # Low Level Controller
            # =============================
            # Calculates throttle position necessary to obtain the desired acceleration

            if a>0:
            	throttle_output = (np.tanh(a) + 1)/2
            else:
            	throttle_output = 0
            brake_output    = 0

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """

            # Using Stanley Controller
            #=========================
            #
            # Steering angle = heading error + angle to eliminate cross track error
            #

            # coefficients
            k = 1
            k_s = 0.001

            # Find heading error
            

            # Trajectory heading
            trajectory_heading = np.arctan2((waypoints[-1][1]-waypoints[0][1]),(waypoints[-1][0]-waypoints[0][0]))
            heading_error = trajectory_heading - yaw
            
            # Limit heading error value range
            if np.abs(heading_error)>np.pi:
            	heading_error = (heading_error - 2*np.pi*np.sign(heading_error))

            # Calculate crosstrack error
            # Min. distance b/w vehicle reference frame and a point on the desired path
            e_crosstrack = np.min(np.sum((np.array([x,y])-np.array(waypoints)[:,:2])**2,axis=1))

            # Yaw angle of crosstrack line
            crosstrack_yaw = np.arctan2((y-waypoints[0][1]),(x-waypoints[0][0]))

            # crosstrack heading compared to desired trajectory
            crosstrack_heading = trajectory_heading-crosstrack_yaw

            # Limit crosstrack heading error value range
            if np.abs(crosstrack_heading)>np.pi:
            	crosstrack_heading = (crosstrack_heading - 2*np.pi*np.sign(crosstrack_heading))

            # Set crosstrack for to point in the right direction
            e_crosstrack = abs(e_crosstrack)*np.sign(crosstrack_heading)

            # slope = (waypoints[0][1]-y)/(waypoints[0][0]-x)
            crosstrack_error_angle = np.arctan(k*e_crosstrack/(k_s+v)) # np.sign(slope)*

            # Limit crosstrack_error_angle value range
            if np.abs(crosstrack_error_angle)>np.pi:
            	crosstrack_error_angle = (crosstrack_error_angle - 2*np.pi*np.sign(crosstrack_error_angle))

            # Max abs(crosstrack_error_angle) value must be pi/2
            if np.abs(crosstrack_error_angle)>np.pi/2:
            	crosstrack_error_angle = 0.5*np.pi*np.sign(crosstrack_error_angle)

            # Update steering output and limit its range
            steer_output = heading_error + crosstrack_error_angle

            if np.abs(steer_output)>np.pi:
            	steer_output = (steer_output - 2*np.pi*np.sign(steer_output))

            if abs(steer_output)>1.22:
            	steer_output = 1.22*np.sign(steer_output)


            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t
        self.vars.v_error_previous = v_error
        self.vars.v_integral_error_previous = v_integral_error
