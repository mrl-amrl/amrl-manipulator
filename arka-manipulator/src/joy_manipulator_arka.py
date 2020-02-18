#!/usr/bin/env python

import sys

import rospy
import yaml
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

from arka_manipulator.cfg import ManipulatorJoyConfig
from arka_manipulator.msg import ManipulatorData
from mercury_common.srv import SetEnabled
from manipulator_protocol import _ManiPulatorProtocol
from mercury import Joy as MercuryJoy
from mercury import logger

node_name = 'arka_manipulator'

UP_DIRECTION = 1
DOWN_DIRECTION = 2
STOP_DIRECTION = 0


class JoyArmArka:
    def __init__(self):

        self.twist_msg = TwistStamped()
        self.init_params()
        self.init_pub_service()
        self.manipulator_protocol = _ManiPulatorProtocol(
            self.controller_ip,
            self.main_ip, self.main_port,
            self.sensor_ip, self.sensor_port)
        self.is_arm_enable = False
        self.semi_status = False
        self.hazmat_pressed = False
        self.arm_controller = MercuryJoy(auto_zero=True)
        self.manipulator_data_msg = ManipulatorData()
        DynamicReconfigureServer(ManipulatorJoyConfig, self._dynamic_params)

        rospy.Service('/mercury/manipulator/enable',
                      SetEnabled, self._set_enable_arm_cb)
        rospy.Service('/mercury/manipulator/set_enable_semi',
                      SetEnabled, self._enable_semi_srv_cb)

        self._semi_callbacks = {'semi_x_axes': self._semi_x_linear_cb,
                                'semi_y_inc_axes': self._semi_y_linear_cb,
                                'semi_y_dec_axes': self._semi_y_linear_cb,
                                'semi_z_axes': self._semi_z_linear_cb,
                                'joint4_inc_btn': self._joint4_inc_cb,
                                'joint4_dec_btn': self._joint4_dec_cb,
                                'joint5_inc_btn': self._joint5_inc_cb,
                                'joint5_dec_btn': self._joint5_dec_cb,
                                'prismatic_axes': self._prismatic_axes,
                                'reset_motors_btn': self._reset_motors_cb,
                                'arm_led_btn': self._arm_led_cb,
                                'sensor_box_dec_btn': self._sensor_box_dec_cb,
                                'sensor_box_inc_btn': self._sensor_box_inc_cb,
                                'hazmat_enable_btn': self._hazmat_cb,
                                'gripper_axes': self._gripper_axes,
                                'enable_disable_semi_btn': self._enable_disable_semi_cb
                                }
        self._manual_callbacks = {'joint1_inc_axes': self._joint1_axes_cb,
                                  'joint1_dec_axes': self._joint1_axes_cb,
                                  'joint2_axes': self._joint2_axes_cb,
                                  'joint3_axes': self._joint3_axes_cb,
                                  'joint4_inc_btn': self._joint4_inc_cb,
                                  'joint4_dec_btn': self._joint4_dec_cb,
                                  'joint5_inc_btn': self._joint5_inc_cb,
                                  'joint5_dec_btn': self._joint5_dec_cb,
                                  'prismatic_axes': self._prismatic_axes,
                                  'reset_motors_btn': self._reset_motors_cb,
                                  'arm_led_btn': self._arm_led_cb,
                                  'sensor_box_dec_btn': self._sensor_box_dec_cb,
                                  'sensor_box_inc_btn': self._sensor_box_inc_cb,
                                  'hazmat_enable_btn': self._hazmat_cb,
                                  'gripper_axes': self._gripper_axes,
                                  'enable_disable_semi_btn': self._enable_disable_semi_cb
                                  }

    def _dynamic_params(self, config, level):
        self._config = config
        logger.log_warn(config['joint1_inc_axes'])
        logger.log_warn(config['joint1_dec_axes'])
        self._max_speeds = {'joint1': config['joint1_max_speed'],
                            'joint2': config['joint2_max_speed'],
                            'joint3': config['joint3_max_speed'],
                            'joint4': config['joint4_max_speed'],
                            'joint5': config['joint5_max_speed'],
                            'gripper': config['gripper_max_speed'],
                            'sensor_box': config['sensor_box_max_speed'],
                            'prismatic': config['prismatic_max_speed']
                            }
        return config

    def init_pub_service(self):
        self.jog_cmd_pub = rospy.Publisher(
            '/jog_server/delta_jog_cmds', TwistStamped, queue_size=1)
        self.manipulator_data_pub = rospy.Publisher(
            '/manipulator/data', ManipulatorData, queue_size=10)

        self.enable_hazmat_service = rospy.ServiceProxy(
            '/blackvenus_tele/set_hazmat_enabled_joy', SetEnabled)
        self.enable_qr_code_service = rospy.ServiceProxy(
            '/mrl_qrcode_detection/set_enable', SetEnabled)

    def init_params(self):
        '''from launch file
        should be String'''
        self.controller_ip = rospy.get_param(
            '~controller_ip', '192.168.10.10')
        self.main_ip = rospy.get_param(
            '~main_board_ip', '192.168.10.170')
        self.sensor_ip = rospy.get_param(
            '~sensor_board_ip', '192.168.10.20')
        self.main_port = rospy.get_param(
            '~main_board_port', '3030')
        self.sensor_port = rospy.get_param(
            '~sensor_board_port', '3033')

    def _set_enable_arm_cb(self, req):
        self.is_arm_enable = req.enabled
        self.arm_controller.unsubscribe_all()
        if req.enabled:
            logger.log_error("arm enable")
            self.arm_controller.subscribe(self._joy_callback)
            for key in self._manual_callbacks:
                logger.log_error(self._config[key])
                if key in ['enable_disable_semi_btn', 'arm_led_btn']:
                    self.arm_controller.on_pressed(
                        self._config[key], self._manual_callbacks[key])
                else:
                    self.arm_controller.on_changed(
                        self._config[key], self._manual_callbacks[key])
        else:
            self.manipulator_protocol.set_zero()
            self.send()
        return req.enabled

    def _joy_callback(self, data):
        # self.send()
        if self.jog_cmd_pub.get_num_connections > 0:
            self.jog_cmd_pub.publish(self.twist_msg)
        if self.manipulator_data_pub.get_num_connections() > 0:
            self.manipulator_data_pub.publish(self.manipulator_data_msg)

    def enable_disable_semi_arm(self):
        if self.semi_status and self.is_arm_enable:
            self.arm_controller.unsubscribe_all()
            self.arm_controller.subscribe(self._joy_callback)
            for keys in self._semi_callbacks:
                if str(keys).endswith('btn'):
                    self.arm_controller.on_changed(
                        self._config[keys], self._semi_callbacks[keys])
                elif str(keys).endswith('axes'):
                    self.arm_controller.on_changed(
                        self._config[keys], self._semi_callbacks[keys])
            logger.log_warn('Semi mode')
        elif not self.semi_status and self.is_arm_enable:
            self.arm_controller.unsubscribe_all()
            self.arm_controller.subscribe(self._joy_callback)
            for keys in self._manual_callbacks:
                if str(keys).endswith('btn'):
                    self.arm_controller.on_changed(
                        self._config[keys], self._manual_callbacks[keys])
                elif str(keys).endswith('axes'):
                    self.arm_controller.on_changed(
                        self._config[keys], self._manual_callbacks[keys])
            logger.log_warn('Manual mode')

    def _enable_semi_srv_cb(self, req):
        self.semi_status = req.enabled
        self.enable_disable_semi_arm()
        return req.enabled

    def get_axes_speed_direction(self, value):
        '''
        return value,direction
        '''

        if value > 0:
            return value, UP_DIRECTION
        elif value == 0:
            return 0, STOP_DIRECTION
        else:
            return abs(value), DOWN_DIRECTION

    def _joint1_axes_cb(self, value, inc_dec=None):
        speed, direction = self.get_axes_speed_direction(value)
        if not inc_dec == None:
            direction = inc_dec
        self.manipulator_protocol.direction.joint1 = direction
        self.manipulator_protocol.angularspeed.joint1 = speed * \
            self._max_speeds['joint1']

        self.manipulator_data_msg.joint1_direction = self.manipulator_protocol.direction.joint1
        self.manipulator_data_msg.joint1_speed = self.manipulator_protocol.angularspeed.joint1

    def _joint2_axes_cb(self, value, inc_dec=None):
        speed, direction = self.get_axes_speed_direction(value)
        if not inc_dec == None:
            direction = inc_dec
        self.manipulator_protocol.direction.joint2 = direction
        self.manipulator_protocol.angularspeed.joint2 = speed * \
            self._max_speeds['joint2']

        self.manipulator_data_msg.joint2_direction = self.manipulator_protocol.direction.joint2
        self.manipulator_data_msg.joint2_speed = self.manipulator_protocol.angularspeed.joint2

    def _joint3_axes_cb(self, value, inc_dec=None):
        speed, direction = self.get_axes_speed_direction(value)
        if not inc_dec == None:
            direction = inc_dec
        self.manipulator_protocol.direction.joint3 = direction
        self.manipulator_protocol.angularspeed.joint3 = speed * \
            self._max_speeds['joint3']

        self.manipulator_data_msg.joint3_direction = self.manipulator_protocol.direction.joint3
        self.manipulator_data_msg.joint3_speed = self.manipulator_protocol.angularspeed.joint3

    def _prismatic_axes(self, value, inc_dec=None):
        _, direction = self.get_axes_speed_direction(value)

        self.manipulator_protocol.direction.prismatic = direction

        self.manipulator_data_msg.prismatic_direction = self.manipulator_protocol.direction.prismatic

    def _joint4_cb(self, inc=None, dec=None):
        value = 0
        if inc:
            value = 1
        elif dec:
            value = -1
        speed, direction = self.get_axes_speed_direction(value)
        self.manipulator_protocol.direction.joint4 = direction
        self.manipulator_protocol.angularspeed.joint4 = speed * \
            self._max_speeds['joint4']

        self.manipulator_data_msg.joint4_direction = self.manipulator_protocol.direction.joint4
        self.manipulator_data_msg.joint4_speed = self.manipulator_protocol.angularspeed.joint4

    def _joint4_inc_cb(self, value, inc_dec=None):
        self._joint4_cb(inc=value)

    def _joint4_dec_cb(self, value, dir=None):
        self._joint4_cb(dec=value)

    def _joint5_cb(self, inc=None, dec=None):
        value = 0
        if inc:
            value = 1
        elif dec:
            value = -1
        speed, direction = self.get_axes_speed_direction(value)
        self.manipulator_protocol.direction.joint5 = direction
        self.manipulator_protocol.angularspeed.joint5 = speed * \
            self._max_speeds['joint5']

        self.manipulator_data_msg.joint5_direction = self.manipulator_protocol.direction.joint5
        self.manipulator_data_msg.joint5_speed = self.manipulator_protocol.angularspeed.joint5

    def _joint5_inc_cb(self, value, inc_dec=None):
        self._joint5_cb(inc=value)

    def _joint5_dec_cb(self, value, dir=None):
        self._joint5_cb(dec=value)

    def _gripper_axes(self, value, inc_dec=None):
        speed, direction = self.get_axes_speed_direction(-value)
        self.manipulator_protocol.direction.gripper = direction
        self.manipulator_protocol.angularspeed.gripper = speed * \
            self._max_speeds['gripper']

        self.manipulator_data_msg.gripper_direction = self.manipulator_protocol.direction.gripper
        self.manipulator_data_msg.gripper_speed = self.manipulator_protocol.angularspeed.gripper

    def _arm_led_cb(self):
        self.manipulator_protocol.set_led.set_led = 1 - \
            self.manipulator_protocol.set_led.set_led
        self.manipulator_data_msg.arm_led = self.manipulator_protocol.set_led.set_led

    def _hazmat_cb(self, value):
        state = True if int(value) == 1 else False
        self.enable_qr_code_service(state)
        self.enable_hazmat_service(state)

    def _reset_motors_cb(self, value):
        self.manipulator_protocol.reset_motor.reset_main_board_motor = int(
            value)
        self.manipulator_protocol.reset_motor.reset_sensor_board_motor = int(
            value)
        self.manipulator_data_msg.reset_motors = self.manipulator_protocol.reset_motor.reset_main_board_motor

    def _sensor_box_cb(self, inc=None, dec=None):
        value = 0
        if inc:
            value = 1
        elif dec:
            value = -1
        speed, direction = self.get_axes_speed_direction(value)
        self.manipulator_protocol.direction.sensor_box = direction
        self.manipulator_protocol.angularspeed.sensor_box = speed * \
            self._max_speeds['sensor_box']

        self.manipulator_data_msg.sensor_box_direction = self.manipulator_protocol.direction.sensor_box
        self.manipulator_data_msg.sensor_box_speed = self.manipulator_protocol.angularspeed.sensor_box

    def _sensor_box_inc_cb(self, value, inc_dec=None):
        self._sensor_box_cb(inc=value)

    def _sensor_box_dec_cb(self, value, dir=None):
        self._sensor_box_cb(dec=value)

    def _semi_status_cb(self, value, inc_dec=None):
        self.semi_status = not self.semi_status
        self.enable_disable_semi_arm()

    def _semi_x_linear_cb(self, value, inc_dec=None):
        speed, direction = self.get_axes_speed_direction(value)
        if not inc_dec == None:
            direction = inc_dec
        self.twist_msg.header.stamp = rospy.Time.now()
        if direction == STOP_DIRECTION:
            self.twist_msg.twist.linear.x = 0
        elif direction == UP_DIRECTION:
            self.twist_msg.twist.linear.x = speed
        elif direction == DOWN_DIRECTION:
            self.twist_msg.twist.linear.x = -speed

        self.jog_cmd_pub.publish(self.twist_msg)

    def _semi_y_linear_cb(self, value, inc_dec=None):
        speed, direction = self.get_axes_speed_direction(value)
        if not inc_dec == None:
            direction = inc_dec
        if direction == STOP_DIRECTION:
            self.twist_msg.twist.linear.y = 0
        elif direction == UP_DIRECTION:
            self.twist_msg.twist.linear.y = speed
        elif direction == DOWN_DIRECTION:
            self.twist_msg.twist.linear.y = -speed

    def _semi_z_linear_cb(self, value, inc_dec=None):
        speed, direction = self.get_axes_speed_direction(value)
        if not inc_dec == None:
            direction = inc_dec
        if direction == STOP_DIRECTION:
            self.twist_msg.twist.linear.z = 0
        elif direction == UP_DIRECTION:
            self.twist_msg.twist.linear.z = speed
        elif direction == DOWN_DIRECTION:
            self.twist_msg.twist.linear.z = -speed

    def _enable_disable_semi_cb(self):
        self.semi_status = not self.semi_status
        self.enable_disable_semi_arm()

    def send(self):
        self.manipulator_protocol.send_data_old_main_board()
        self.manipulator_protocol.send_sensor_board_manipulator_data()
