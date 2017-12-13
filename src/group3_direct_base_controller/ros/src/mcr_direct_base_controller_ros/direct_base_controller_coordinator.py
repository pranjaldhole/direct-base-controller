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
from transform_to_pose_converter import TransformToPoseConverter
from component_wise_pose_error_calculator import ComponentWisePoseErrorCalculator
from twist_controller import TwistController
from twist_limiter import TwistLimiter


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

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 100.0))

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)

        self.pose_error_monitor_event_in = rospy.Publisher(
            '~pose_error_monitor_event_in', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.twist_synchronizer_event_in = rospy.Publisher(
            '~twist_synchronizer_event_in', std_msgs.msg.String, latch=True, queue_size=1
        )
        self.base_twist = rospy.Publisher('~twist', geometry_msgs.msg.Twist, queue_size=1)

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~pose_monitor_feedback", mcr_monitoring_msgs.msg.ComponentWisePoseErrorMonitorFeedback,
                         self.pose_monitor_feedback_cb)
        rospy.Subscriber('~pose_2', geometry_msgs.msg.PoseStamped, self.pose_2_cb)

        # Setup for component_wise_pose_error_calculator
        # TODO - publisher should be removed afer pose_error_monitor is merged
        self.pose_error = rospy.Publisher(
            '~pose_error', mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference, queue_size=5
        )
        # TODO - Remove after twist_synchronizer is merged
        self.limited_twist = rospy.Publisher(
            '~limited_twist', geometry_msgs.msg.TwistStamped, queue_size=5
        )

        self.transform_to_pose_converter = TransformToPoseConverter();
        self.component_wise_pose_error_calculator = ComponentWisePoseErrorCalculator();
        self.twist_controller = TwistController();
        self.twist_limiter = TwistLimiter();

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

        if self.pose_monitor_feedback is not None:
            if self.pose_monitor_feedback.is_linear_x_within_tolerance and \
                    self.pose_monitor_feedback.is_linear_y_within_tolerance and \
                    self.pose_monitor_feedback.is_angular_z_within_tolerance:
                self.send_event_to_components("stop")
                self.event_out.publish('e_success')
                self.event = None
                self.pose_monitor_feedback = None
                self.started_components = False

                self.publish_zero_velocities()

                return 'INIT'

        # Get converted pose and calculate the pose error and publish
        converted_pose = self.transform_to_pose_converter.get_converted_pose()
        if(converted_pose != None):
            pose_error = self.component_wise_pose_error_calculator.get_component_wise_pose_error(converted_pose, self.pose_2)
            if (pose_error !=None):
                self.pose_error.publish(pose_error) # TODO - Stop publishing after pose_error_monitor is merged

                cartesian_velocity = self.twist_controller.get_cartesian_velocity(pose_error)
                if cartesian_velocity:
                    limited_twist = self.twist_limiter.get_limited_twist(cartesian_velocity)
                    if limited_twist:
                        self.limited_twist.publish(limited_twist) # TODO Remove after merging Twist synchronizer

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
            self.twist_synchronizer_event_in.publish('e_start')

            self.started_components = True

        elif event == 'stop':
            self.transform_to_pose_converter.event = 'e_stop'
            self.component_wise_pose_error_calculator.monitor_event = 'e_stop'

            self.pose_error_monitor_event_in.publish('e_stop')
            self.twist_synchronizer_event_in.publish('e_stop')

            self.started_components = False


def main():
    rospy.init_node("direct_base_controller", anonymous=True)
    direct_base_controller_coordinator = DirectBaseControllerCoordinator()
    direct_base_controller_coordinator.start()
