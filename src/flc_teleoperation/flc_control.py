#!/usr/bin/env python
"""
Free Look Control (FLC) using ActionServer, control base and camera.

When enabled, convert joystick input to FLC control for base and camera.
An ActionServer enables easy enable/disable.

Subscribes to topics: joy, odom, input_pan, input_tilt
Publishes to: output_base, output_pan, output_tilt (angles in radians!)
"""

from __future__ import print_function

from math import pi, cos, sin, degrees, radians
import time
import tf
import rospy
import actionlib
import std_msgs.msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import geometry_msgs.msg
import flc_teleoperation.msg


class FLCClass(object):
    """
    Class for FLC.
    """
    ## Use this to setup customized publishing.
    customized_publishing = False

    def __init__(self, rate):
        """
        Constructor of FLCClass

        @param rate Rate to run at (e.g. 10 Hz).
        """
        ## Rate to run at, e.g. 10Hz.
        self.rate = rate

        # Instantiate subclasses for managing data.
        self.manager_flc = self.FLCManager(self.rate)
        self.manager_input = self.InputManager(self.customized_publishing)
        self.manager_output = self.OutputManager(self.customized_publishing)

    def run(self):
        """
        Main loop of FLC.

        Calculate and publish control while timeout has not been reached.
        Assume running at self.rate Hz, for calculations.

        @returns Bool representing if FLC is enabled (publishing) or not.
        """
        ## Get latest data.
        data_ = self.manager_input.get_data()
        if self.customized_publishing:
            pass

        ## Keep track of current state.
        self.manager_flc.update_state(data_)

        # Only calculate and publish in case controller is enabled.
        if data_['enabled']:
            if data_['reset']:
                rospy.logdebug(rospy.get_name() + " Reset FLC.")
                self.zero()
            ## Calculate control signal
            control_ = self.manager_flc.calculate_control(data_)

            ## Update new state.
            self.manager_flc.update_state(data_, control_, self.rate)

            if self.customized_publishing:
                pass

            ## Publish data
            self.manager_output.publish(control_, data_['offset'])
            return True
        else:
            self.manager_output.reset()
            return False

    def shutdown(self):
        """
        Preparations for shutdown, such as publishing zero velocities.
        """
        self.zero()

    def zero(self):
        """
        Make sure no movement is performed.
        """
        self.manager_input.reset()
        self.manager_flc.reset()
        self.manager_output.reset()
        rospy.logdebug(rospy.get_name() + " Publish zero velocities.")

    class FLCManager(object):
        """
        Manages all calculations of control.
        """

        rate = 1.0  # Publishing rate, for calculations. Set in constructor.
        ## Current state, in radians.
        state = {'phi': 0.0, 'theta': 0.0, 'rotation': 0.0}

        # For feedback.
        imu_prev = 0.0
        imu_now = 0.0
        rot_est = 0.0

        def __init__(self, rate_):
            """
            Constructor
            """
            self.rate = rate_
            ## Physical variables
            self.data = {'L': rospy.get_param('~ugv_L', 0.15),
                         'd': rospy.get_param('~ugv_d', 0.42),
                         'vel_max': rospy.get_param('~vel_max', 0.3),
                         'ang_max': rospy.get_param('~ang_max', 0.3),
                         'scale_tilt': rospy.get_param('~scale_tilt', 0.25)}

        def update_state(self, data_, control_=None, rate_=None):
            """
            Update current state.
            """
            if data_['camera'] is not None:
                self.state['phi'] = data_['camera'][0]
                self.state['theta'] = data_['camera'][1]
            if control_ is not None and rate_ is not None:
                self.state['phi'] += control_['camera_delta'][0]*(1.0/rate_)
                self.state['theta'] += control_['camera_delta'][1]*(1.0/rate_)
                self.state['rotation'] += control_['base'][1]*(1.0/rate_)

        def reset(self):
            """
            Reset data
            """
            self.state['phi'] = 0.0
            self.state['theta'] = 0.0
            self.imu_prev = 0.0
            self.imu_now = 0.0
            self.rot_est = 0.0

        def calculate_control(self, data_):
            """
            Calculates and returns control signal for base and camera.

            @param data_ Dict with array 'joy' (joystick), 'odom' etc.

            @return
                control signal for base and camera.
            """
            ## Store current rotation from odometry (possibly with IMU data).
            self.imu_now = self.state['rotation']  #: Same as stored if no data.
            if data_['odom'] is not None:
                self.imu_now = ((tf.transformations.euler_from_quaternion(data_['odom'][3:]))[2])
            delta_rot_err = self.rot_est - (self.imu_now - self.imu_prev)
            # rospy.loginfo("Correction: %f", delta_rot_err)

            control_ = self.input_to_flc(data_['joy'][0], data_['joy'][1],
                                         data_['joy'][2], data_['joy'][3])
            ## Convert left/right track vel. to platform movement.
            control_data = {
                'base': [(control_[0] + control_[1]) / 2.0,
                         (control_[0] - control_[1]) / self.data['d']],
                'camera': [self.state['phi'] + control_[2] + delta_rot_err,
                           self.state['theta'] + control_[3]],
                'base_delta': [self.state['rotation'] + delta_rot_err],
                'camera_delta': [control_[2], control_[3]]
            }
            self.rot_est = control_data['base'][1]*1.0/self.rate
            self.imu_prev = self.imu_now
            return control_data

        def input_to_flc(self, js_l_ud, js_l_lr, js_r_ud, js_r_lr):
            """
            Calculate control using FLC.

            @param v1 Left stick up/down (forward/backward motion)
            @param v2 Left stick left/right (left/right translation)
            @param w_ Right stick left/right (camera rotation)
            @param t_ Right stick up/down (camera tilt)

            @returns array with left, right track vel., camera pan, tilt.
            """
            # Get variables, for shorter notation in equations.
            vel_max = self.data['vel_max']
            ang_max = self.data['ang_max']
            d = self.data['d']
            L = self.data['L']
            phi = self.state['phi']
            theta = self.state['theta']
            v1 = js_l_ud  #: Left joystick, up/down
            v2 = js_l_lr  #: Left joystick, left/right
            w_ = js_r_lr  #: Right joystick, left/right
            tilt_ = -js_r_ud  #: Right joystick, up/down

            if d == 0:  #: Make sure base width is valid.
                rospy.logerr("Incorrect width of UGV.")
                return [0, 0, 0, 0]
            if L == 0:  #: Make sure camera offset is valid.
                rospy.logerr("Camera offset invalid, would divide by zero.")
                return [0, 0, 0, 0]
            if v1 < 0:  #: When moving backwards.
                phi = -pi - phi
                v1 = -v1
            # Velocities on left and right track
            v_l = ((v1 - v2*d/(2*L)) * cos(phi) - (v2 + v1*d/(2*L)) * sin(phi))*vel_max
            v_r = ((v1 + v2*d/(2*L)) * cos(phi) - (v2 - v1*d/(2*L)) * sin(phi))*vel_max
            # Modifications here to limit rotational speed.
            ang_speed = abs((v_r - v_l) / d)  # Calculate angular speed.
            if ang_speed > ang_max:  # Scale v_r and v_l
                rotation_scale_factor = ang_max / ang_speed
                v_l *= rotation_scale_factor
                v_r *= rotation_scale_factor
            # End of modifications to limit rotational speed.
            # Reduce linear speed
            high_des_track_speed = max(abs(v_l), abs(v_r))
            if high_des_track_speed > vel_max:
                linear_scale_factor = vel_max / high_des_track_speed
                v_l = v_l * linear_scale_factor
                v_r = v_r * linear_scale_factor
            k = (w_ - (v_r - v_l) / d)*(1.0/self.rate)
            tilt_ *= self.data['scale_tilt']
            ## Return data
            return v_r, v_l, k, tilt_

    class InputManager(object):
        """
        Gathers all external data.

        Should contain all required subscribers for FLC, e.g. joy and odom.
        """
        ## Keep track if publishing should be enabled
        is_enabled_state = True

        ## Latest Joy message and timestamp
        msg_joy = {'msg': Joy(), 'time': rospy.Time(0)}

        ## Latest Odometry message and timestamp
        msg_odom = {'msg': Odometry(), 'time': rospy.Time(0)}

        ## Latest Camera message and timestamp
        msg_camera = {'msg': None, 'time': rospy.Time(0)}

        offset = [radians(-36.0), radians(90.0)]  #: Pan, Tilt.

        def __init__(self, customized_publishing):
            self.customized_publishing = customized_publishing
            self.offset = [
                radians(rospy.get_param('~pan_offset', -36.0)),
                radians(rospy.get_param('~tilt_offset', 90.0))
            ]
            ## Subscriber for Joy messages
            joy_topic = rospy.get_param('~joy_topic', "flc_joy")
            _ = rospy.Subscriber(joy_topic, Joy, self.callback_joy, queue_size=None)

            ## Subscriber for Odom messages
            odom_topic = rospy.get_param('~odom_topic', "odom")
            _ = rospy.Subscriber(odom_topic, Odometry, self.callback_odom, queue_size=None)

            if self.customized_publishing:
                pass

        def reset(self):
            """
            Resets latest message, to avoid movement.
            """
            self.msg_joy['msg'] = Joy()
            self.msg_joy['time'] = rospy.Time(0)
            self.msg_odom['time'] = rospy.Time(0)

            if self.customized_publishing:
                pass

        def callback_joy(self, joy):
            """
            Callback for Joy messages.

            Stores message and time when it was stored.

            @param joy Incoming Joy message.
            """
            self.msg_joy['msg'] = joy
            self.msg_joy['time'] = rospy.Time.now()

        def callback_odom(self, odom):
            """
            Callback for Odometry messages.

            Stores message and time when it was stored.

            @param odom Incoming Odometry message.
            """
            self.msg_odom['msg'] = odom
            self.msg_odom['time'] = rospy.Time.now()

        def callback_camera(self, msg):
            """
            Callback for Camera messages.

            Stores camera message and time when it was stored.

            @param msg Incoming camera message.
            """
            self.msg_camera['msg'] = msg
            self.msg_camera['time'] = rospy.Time.now()

        def get_data(self):
            """
            Returns input data, converted to suitable output format.

            @returns Input data as dict (joy, odom, camera, enabled, offset),
                containing extracted data. Offset has been removed from camera.
            """
            ## Prevent publishing on old data (>=1s) or incorrect joy input.
            self.is_enabled_state = False
            reset_flc = False
            if (rospy.Time.now()-self.msg_joy['time']).to_sec() < 1.0:
                if len(self.msg_joy['msg'].buttons) > 3:
                    if self.msg_joy['msg'].buttons[4]:  #: Deadmans button.
                        self.is_enabled_state = True
                        rospy.logdebug(rospy.get_name() + ": State is enabled.")
                    if len(self.msg_joy['msg'].buttons) > 10:
                        if self.msg_joy['msg'].buttons[10]:  #: Reset FLC.
                            reset_flc = True
                else:
                    rospy.logdebug(rospy.get_name() + ": Disabled due to incorrect joy msg.")
            else:
                rospy.logdebug(rospy.get_name() + ": Disabled due to old message.")
            ##: Left joystick up/down, left/right; Right up/down, left/right.
            data_joy = [0.0, 0.0, 0.0, 0.0]
            if len(self.msg_joy['msg'].axes) >= 4:  #: Only if input size is ok
                data_joy = [self.msg_joy['msg'].axes[1],
                            self.msg_joy['msg'].axes[0],
                            self.msg_joy['msg'].axes[4],
                            self.msg_joy['msg'].axes[3]]
            data_odom = None
            if self.msg_odom['time'] != rospy.Time(0):
                data_odom = [self.msg_odom['msg'].pose.pose.position.x,
                             self.msg_odom['msg'].pose.pose.position.y,
                             self.msg_odom['msg'].pose.pose.position.z,
                             self.msg_odom['msg'].pose.pose.orientation.x,
                             self.msg_odom['msg'].pose.pose.orientation.y,
                             self.msg_odom['msg'].pose.pose.orientation.z,
                             self.msg_odom['msg'].pose.pose.orientation.w]
            data_camera = None
            if self.msg_camera['time'] != rospy.Time(0):
                pan = radians(self.msg_camera['msg'].pan) - self.offset[0]
                tilt = radians(self.msg_camera['msg'].tilt) - self.offset[1]
                data_camera = [pan, tilt]
            return {'joy': data_joy, 'odom': data_odom, 'camera': data_camera,
                    'enabled': self.is_enabled_state, 'offset': self.offset,
                    'reset': reset_flc}

    class OutputManager(object):
        """
        Class for publishing both to base and camera.
        """
        ## If True, enable publishing. If false, don't.
        enabled = True
        ## Keep track on if zero publish has taken place.
        was_reset = False

        def __init__(self, customized_publishing):
            """
            Constructor

            @param customized_publishing use e.g. multiplexer.
            """
            ## Instance of BaseClass, managing output to base (UGV).
            self.base = self.BaseClass(customized_publishing)
            ## Instance of CameraClass, managing output to camera.
            self.camera = self.CameraClass(customized_publishing)

        def reset(self):
            """
            Reset settings, if needed.
            """
            if not self.was_reset:
                self.was_reset = True
                self.base.reset()
                self.camera.reset()

        def publish(self, control, offset):
            """
            Sends data to both base and camera, for publishing.

            Input should be a dict with 'base', 'camera', both arrays.
            base[0] is linear velocity, base[1] angular velocity.
            camera[0] camera pan, camera[1] tilt.

            @param control Control signal to publish (dict) {'base', 'camera']
            @param offset Offset of camera.
            """
            if self.enabled:  #: Check if publishing should be enabled.
                self.was_reset = False
                self.base.publish(control)
                self.camera.publish(control, offset)

        def enable_publish(self, state):
            """
            Enable or disable publishing. True for enabled, false for disabled

            @param state Bool, True for enabled, False for disabled publishing
            """
            self.enabled = state

        class BaseClass(object):
            """
            Class for publishing to base.
            """
            def __init__(self, customized_publishing):
                self.customized_publishing = customized_publishing
                self.output_base = rospy.get_param('~output_base', '/teleop_joy/cmd_vel')
                self.pub = rospy.Publisher(self.output_base, geometry_msgs.msg.Twist, queue_size=0)
                if self.customized_publishing:
                    pass

            def publish(self, control):
                """
                Publish command to base.

                @param control Data to publish, could need conversion.
                """
                ## Message to be published.
                msg = geometry_msgs.msg.Twist()
                # Insert data.
                msg.linear.x = control['base'][0]
                msg.angular.z = control['base'][1]
                # Publish.
                self.pub.publish(msg)

            def reset(self):
                """
                Publish zero command.
                """
                self.pub.publish(geometry_msgs.msg.Twist())

        class CameraClass(object):
            """
            Class for publishing to camera.
            """
            pub_pan = None
            pub_tilt = None
            pub_cam = None
            cam_msg = None

            def __init__(self, customized_publishing):
                self.customized_publishing = customized_publishing
                camera_pan = rospy.get_param('~camera_pan_topic', 'camera_pan')
                self.pub_pan = rospy.Publisher(camera_pan, std_msgs.msg.Float32, queue_size=0)
                camera_tilt = rospy.get_param('~camera_tilt_topic', 'camera_tilt')
                self.pub_tilt = rospy.Publisher(camera_tilt, std_msgs.msg.Float32, queue_size=0)
                if self.customized_publishing:
                    pass

            def publish(self, control, offset):
                """
                Publish command to camera.

                Convert pan and tilt from 'control' to movement of camera.

                @param control Data to publish, could need conversion.

                @param offset Camera pan/tilt offset.
                """
                pan_r = control['camera'][0]
                tilt_r = control['camera'][1]
                ## Standard publishing.
                self.pub_pan.publish(pan_r)
                self.pub_tilt.publish(tilt_r)
                if self.customized_publishing:
                    pass

            def reset(self):
                """
                Publish zero command.
                """
                pass


class FLCActionServer(object):
    """
    Wraps FLC in an ActionServer, for easier enable/disable.

    """
    def __init__(self, name, rate):
        """
        Constructor

        @param name Name of the actionserver.
        @param rate Rate to run the actionserver at, used for FLC calculations.

        """
        ## Rate to run at, needed for FLC calculations.
        self.rate = rospy.Rate(rate)
        ## Register hook to stop movement on shutdown.
        rospy.on_shutdown(self.hook_on_shutdown)
        # Setup ActionServer.
        self._as = actionlib.SimpleActionServer(
            name,
            flc_teleoperation.msg.FLCAction,
            execute_cb=self.execute_cb,
            auto_start=False)
        ## Instance of FLCClass
        self.flc = FLCClass(rate)
        # Start ActionServer after setup.
        self._as.start()

    def execute_cb(self, goal):
        """
        Loop for when ActionServer has a goal and is not preempted.

        Calls FLCClass.run at pre-specified rate (e.g. 10 Hz) until AS stops.

        @param goal If goal.run is True FLC will run, otherwise it will not.
        """
        rospy.logdebug(rospy.get_name() + " Goal: \n%s", goal)
        if goal.run:
            while not rospy.is_shutdown():  #: For terminating with ctrl+c.
                if self._as.is_preempt_requested():
                    rospy.logdebug(rospy.get_name() + " Preempted")
                    self.flc.zero()
                    break
                feedback = self.flc.run()
                self._as.publish_feedback(flc_teleoperation.msg.FLCFeedback(feedback))
                self.rate.sleep()
        self._as.set_succeeded(flc_teleoperation.msg.FLCResult(True))

    def hook_on_shutdown(self):
        """
        Execute additional tasks on shutdown,
        such as publishing zero velocities.
        """
        self.flc.shutdown()


def main_as():
    """
    Launch FLC with ActionServer.
    """
    rospy.init_node('flc')
    try:
        run_hz = 20
        _ = FLCActionServer(rospy.get_name(), run_hz)  #: Initialize AS.
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


def main():
    """
    Launch normally (no ActionServer).
    """
    rospy.init_node('flc')
    try:
        run_hz = 10
        rate_ = rospy.Rate(run_hz)
        flc = FLCClass(run_hz)
        while not rospy.is_shutdown():
            flc.run()
            rate_.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    """
    Entry point.
    """
    try:
        # main_as()  #: Using ActionServer for FLC
        main()  #: Constantly running FLC (no ActionServer)
    except rospy.ROSInterruptException:
        pass
