#!/usr/bin/env python
"""
This module contains a component that calculates
the component-wise error between two poses.

"""
#-*- encoding: utf-8 -*-
__author__ = 'ramit'

import rospy
import std_msgs.msg
import mcr_manipulation_msgs.msg
import mcr_monitoring_msgs.msg
import dynamic_reconfigure.server
import mcr_pose_monitor.cfg.ComponentWisePoseErrorMonitorConfig as ComponentWisePoseErrorMonitorConfig

class ComponentWisePoseErrorMonitor(object):
    """
    Calculates the error between two poses in three
    linear components and three angular components.

    """
    def __init__(self):
        self.has_pose_error_data = False
        self.feedback = mcr_monitoring_msgs.msg.ComponentWisePoseErrorMonitorFeedback()
       

        # node cycle rate (in hz)
        



        # subscribers
        


     def set_parameters(self,threshold_linear_x,threshold_linear_y,\
        threshold_linear_z,threshold_angular_x,threshold_angular_y,threshold_angular_z):
        self.threshold_linear_x = threshold_linear_x
        self.threshold_linear_y = threshold_linear_y
        self.threshold_linear_z = threshold_linear_z
        self.threshold_angular_x = threshold_angular_x
        self.threshold_angular_y = threshold_angular_y
        self.threshold_linear_z = threshold_angular_z
  
        
   


    

        

    


   


    def isComponentWisePoseErrorWithinThreshold(self,pose_error):
        feedback.is_linear_x_within_tolerance = math.abs(pose_error.linear.x) < self.threshold_linear_x
        feedback.is_linear_y_within_tolerance = math.abs(pose_error.linear.y) < self.threshold_linear_y
        feedback.is_linear_z_within_tolerance = math.abs(pose_error.linear.z) < self.threshold_linear_z
        feedback.is_angular_x_within_tolerance = math.abs(pose_error.angular.x) < self.threshold_angular_x
        feedback.is_angular_y_within_tolerance = math.abs(pose_error.angular.y) < self.threshold_angular_y
         feedback.is_angular_z_within_tolerance = math.abs(pose_error.angular.z) < self.threshold_angular_z
        return (feedback.is_linear_x_within_tolerance && feedback.is_linear_y_within_tolerance\
           && feedback.is_linear_z_within_tolerance && feedback.is_angular_x_within_tolerance &&\
            feedback.is_angular_z_within_tolerance)


