#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This component moves the mobile base in Cartesian space until a pose is reached.

It uses the following nodes:
  * (mcr_geometric_relation_monitors) component_wise_pose_error_monitor.
  * (mcr_twist_synchronizer) twist_synchronizer.

The component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

"""

import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_monitoring_msgs.msg
import mcr_manipulation_msgs.msg
import mcr_monitoring_msgs.msg
import dynamic_reconfigure.server
from transform_to_pose_converter import TransformToPoseConverter
from component_wise_pose_error_calculator import ComponentWisePoseErrorCalculator
from component_wise_pose_error_monitor import ComponentWisePoseErrorMonitor
from twist_controller import TwistController
from twist_limiter import TwistLimiter
from twist_synchronizer import TwistSynchronizer
import dynamic_reconfigure.server
import mcr_pose_monitor.cfg.ComponentWisePoseErrorMonitorConfig as ComponentWisePoseErrorMonitorConfig


class DirectBaseControllerCoordinator(object):
    """
    Components that move the base until a pose is reached.

    """
    def __init__(self):
        # params
        self.started_components = False
        self.event = None
        self.pose_monitor_feedback = None
        self.pose_2 = None
        self.has_pose_error_data = False

        self.transform_to_pose_converter = TransformToPoseConverter();
        self.component_wise_pose_error_calculator = ComponentWisePoseErrorCalculator();
        self.component_wise_pose_error_monitor = ComponentWisePoseErrorMonitor();
        self.feedback = mcr_monitoring_msgs.msg.ComponentWisePoseErrorMonitorFeedback()
        
        self.twist_controller = TwistController();
        self.twist_limiter = TwistLimiter();
        self.twist_synchronizer = TwistSynchronizer();

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 100.0))
        self.near_zero = rospy.get_param('~near_zero', 0.001)

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)

        self.dynamic_reconfigure_server = dynamic_reconfigure.server.Server(
            ComponentWisePoseErrorMonitorConfig, self.dynamic_reconfigure_cb
        )

        self.pose_error_monitor_event_in = rospy.Publisher(
            '~pose_error_monitor_event_in', std_msgs.msg.String, latch=True, queue_size=1
        )

        self.base_twist = rospy.Publisher('~twist', geometry_msgs.msg.Twist, queue_size=1)

        self.synchronized_twist = rospy.Publisher('~synchronized_twist', geometry_msgs.msg.TwistStamped, queue_size=5)

        # Setup for component_wise_pose_error_calculator
        # TODO - publisher should be removed afer pose_error_monitor is merged
        self.pose_error = rospy.Publisher(
            '~pose_error', mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference, self.pose_error_cb
        )

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~pose_monitor_feedback", mcr_monitoring_msgs.msg.ComponentWisePoseErrorMonitorFeedback,
                         self.pose_monitor_feedback_cb)
        rospy.Subscriber('~pose_2', geometry_msgs.msg.PoseStamped, self.pose_2_cb)

        rospy.Subscriber('~pose_error', mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference, self.pose_error_cb)
    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

# callbacks for component_wise_pose_error_calculator node----
    def pose_2_cb(self, msg):
        """
        Obtains the second pose.

        """
        self.pose_2 = msg

    def pose_error_cb(self, msg):
        self.pose_error = msg
        self.has_pose_error_data = True
#-------------------------------------------------------------

    def pose_monitor_feedback_cb(self, msg):
        """
        Obtains the feedback from the pose error monitor component

        """
        self.pose_monitor_feedback = msg

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            self.send_event_to_components("start")
            self.event = None
            return 'RUNNING'
        else:
            return 'INIT'


    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """

        if self.event == 'e_stop':
            self.send_event_to_components("stop")
            self.event_out.publish('e_stopped')
            self.event = None
            self.pose_monitor_feedback = None
            self.started_components = False

            self.publish_zero_velocities()

            return 'INIT'

        # Get converted pose and calculate the pose error and publish
        converted_pose = self.transform_to_pose_converter.get_converted_pose()
        if(converted_pose != None):
            pose_error = self.component_wise_pose_error_calculator.get_component_wise_pose_error(converted_pose, self.pose_2)
            if isComponentWisePoseErrorWithinThreshold(pose_error):
                self.send_event_to_components("stop")
                self.event_out.publish('e_success')
                self.event = None
                self.pose_monitor_feedback = None
                self.started_components = False
                publish_zero_velocities()
                return 'INIT'
            else:
                cartesian_velocity = self.twist_controller.get_cartesian_velocity(pose_error)
                if cartesian_velocity:
                    limited_twist = self.twist_limiter.get_limited_twist(cartesian_velocity)
                    if (limited_twist != None):
                        synchronized_twist = self.twist_synchronizer.synchronize_twist(limited_twist, pose_error)
                        if (synchronized_twist != None):
                            self.synchronized_twist.publish(synchronized_twist)
                        #    self.event_out.publish('e_success')
                        #else:
                        #    self.event_out.publish('e_failure')
                        self.twist_synchronizer.reset_component_data()

        return 'RUNNING'

    def publish_zero_velocities(self):
        zero_twist = geometry_msgs.msg.Twist()

        self.base_twist.publish(zero_twist)

    def send_event_to_components(self, event):
        """
        Starts or stops the necessary components

        :param event: The event that determines either to start or stop the components.
        :type event: str

        """
        if event == 'start' and not (self.started_components):
            self.transform_to_pose_converter.event = 'e_start'
            self.component_wise_pose_error_calculator.monitor_event = 'e_start'

            self.pose_error_monitor_event_in.publish('e_start')

            self.started_components = True

        elif event == 'stop':
            self.transform_to_pose_converter.event = 'e_stop'
            self.component_wise_pose_error_calculator.monitor_event = 'e_stop'

            self.pose_error_monitor_event_in.publish('e_stop')

            self.started_components = False

def dynamic_reconfigure_cb(self, config, level):
        """
        Obtains an update for the dynamic reconfigurable parameters.

        """
        self.threshold_linear_x = config.threshold_linear_x
        self.threshold_linear_y = config.threshold_linear_y
        self.threshold_linear_z = config.threshold_linear_z
        self.threshold_angular_x = config.threshold_angular_x
        self.threshold_angular_y = config.threshold_angular_y
        self.threshold_linear_z = config.threshold_angular_z
        return config

            
