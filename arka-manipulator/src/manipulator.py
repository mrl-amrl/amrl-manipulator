#!/usr/bin/env python
from math import pi

import rospy
from std_msgs.msg import UInt8
from trajectory_msgs.msg import JointTrajectory
from joy_manipulator_arka import JoyArmArka
from joint_state_pub import JointStatePublisher
from mercury_common.srv import SetEnabled
from joint_state_pub import JointStatePublisher
from mercury import logger
# from joy_manipulator import ManipulatorJoyCommander

node_name = 'manipulator'
STOP_DIRECTION = 0
UP_DIRECTION = 1
DOWN_DIRECTION = 2


class Manipulator:
    def __init__(self):
        self.is_manip_enable = True
        self.is_semi_enable = False
        self.manipulator_joy_commander = JoyArmArka()
        self.joint_state_pub = JointStatePublisher()
        # self.disable_all_joint()
        self.get_manip_params()
        self.init_services()

        rospy.Subscriber('manipulator_mode', UInt8,
                         callback=self.home_position_cb)
        rospy.Subscriber('/joint_path_command', JointTrajectory,
                         callback=self.moveit_commander)
        rospy.Subscriber('/arka_controller/command',
                         JointTrajectory, callback=self.jog_arm_commander)
        rospy.Service('/mercury/power/arm_led',
                      SetEnabled, self._enable_arm_led_srv)

    def get_manip_params(self):
        self.joints_ratio = {}
        for i in range(1, 5):
            self.joints_ratio['joint{}'.format(i)] = rospy.get_param(
                node_name + '/joint{}/ratio'.format(i))
        logger.log_error(self.joints_ratio)
        self.is_semi_enable = rospy.get_param(
            node_name + '/is_semi_enable', False)

    def init_services(self):
        rospy.Service('/mrl_manipulator_karo/set_emergency_stop_ocu',
                      SetEnabled, self.manip_emergency_cb)

    def manip_emergency_cb(self, req):
        self.manipulator_joy_commander.manipulator_protocol.emergency_stop.emergency_stop = 1 if req.enabled else 0

    def home_position_cb(self, msg):
        self.manipulator_joy_commander.manipulator_protocol.home_position.home_position = msg.data

    def _enable_arm_led_srv(self, req):
        self.manipulator_joy_commander.manipulator_protocol.set_led.set_led = 1 if req.enabled else 0
        return req.enabled

    def stop_semi_joints(self):
        self.manipulator_joy_commander.manipulator_protocol.direction.joint1 = 0
        self.manipulator_joy_commander.manipulator_protocol.angularspeed.joint1 = 0
        self.manipulator_joy_commander.manipulator_protocol.direction.joint2 = 0
        self.manipulator_joy_commander.manipulator_protocol.angularspeed.joint2 = 0
        self.manipulator_joy_commander.manipulator_protocol.direction.joint3 = 0
        self.manipulator_joy_commander.manipulator_protocol.angularspeed.joint3 = 0

    def moveit_commander(self, msg_list):
        rospy.logerr(msg_list)
        if self.manipulator_joy_commander.is_arm_enable and self.manipulator_joy_commander.semi_status:
            # if not self.is_manip_enable:
            # msg_list = JointTrajectory()

            # rospy.logwarn('moveit command recived            {}'.format(msg_list.velocities))
            for index, msg in enumerate(msg_list.points[1:]):
                index += 1
                # rospy.logwarn('moveit command recived            {},index: {}'.format(msg,index))
                _sleep_time = 0.0
                _current_speed_dic = {}
                # msg = JointTrajectory()
                _current_speed_dic['joint1'] = self.rad_per_secn_rpm(
                    msg.velocities[0]) * self.joints_ratio.get('joint1')
                _current_speed_dic['joint2'] = self.rad_per_secn_rpm(
                    msg.velocities[1]) * self.joints_ratio.get('joint2')
                _current_speed_dic['joint3'] = self.rad_per_secn_rpm(
                    msg.velocities[2]) * self.joints_ratio.get('joint3')
                rospy.logwarn('msg V {} ,current V {}'.format(
                    msg.velocities, _current_speed_dic))
                # joint1
                if _current_speed_dic.get('joint1') == 0:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint1 = STOP_DIRECTION
                elif _current_speed_dic.get('joint1') < 0:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint1 = DOWN_DIRECTION
                else:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint1 = UP_DIRECTION

                self.manipulator_joy_commander.manipulator_protocol.angularspeed.joint1 = abs(
                    _current_speed_dic.get('joint1'))
                # joint2
                if _current_speed_dic.get('joint2') == 0:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint2 = STOP_DIRECTION
                elif _current_speed_dic.get('joint2') > 0:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint2 = UP_DIRECTION
                else:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint2 = DOWN_DIRECTION

                self.manipulator_joy_commander.manipulator_protocol.angularspeed.joint2 = abs(
                    _current_speed_dic.get('joint2'))
                # joint3
                if _current_speed_dic.get('joint3') == 0:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint3 = STOP_DIRECTION
                elif _current_speed_dic.get('joint3') > 0:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint3 = DOWN_DIRECTION
                else:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint3 = UP_DIRECTION
                rospy.logwarn('sleep {},sleep time is {},time from start {}'.format((msg.time_from_start.secs + msg.time_from_start.nsecs * 1e-9),
                                                                                    (msg_list.points[index - 1].time_from_start.secs + msg_list.points[index - 1].time_from_start.nsecs * 1e-9), msg_list.points[index - 1].time_from_start.secs))
                self.manipulator_joy_commander.manipulator_protocol.angularspeed.joint3 = abs(
                    _current_speed_dic.get('joint3'))
                _sleep_time = (msg.time_from_start.secs + msg.time_from_start.nsecs * 1e-9) - (
                    msg_list.points[index - 1].time_from_start.secs + msg_list.points[index - 1].time_from_start.nsecs * 1e-9)
                a = rospy.Duration(secs=_sleep_time)
                rospy.sleep(a)
                self.display_data()
                # self.stop_semi_joints()

    def jog_arm_commander(self, msg_list):

        # msg_list = JointTrajectory()
        # rospy.logwarn('moveit command recived            {}'.format(len(msg_list.points)))
        if self.manipulator_joy_commander.is_arm_enable and self.manipulator_joy_commander.semi_status:
            # if not self.is_manip_enable:
            # msg_list = JointTrajectory()

            # rospy.logwarn('moveit command recived in 11            {}'.format(len(msg_list.points)))
            for index, msg in enumerate(msg_list.points):

                # rospy.logwarn('moveit command recived in 22          {},index: {}'.format(msg,index))
                _sleep_time = 0.0
                _current_speed_dic = {}
                # msg = JointTrajectory()
                _current_speed_dic['joint1'] = self.rad_per_secn_rpm(
                    msg.velocities[0]) * self.joints_ratio.get('joint1')
                _current_speed_dic['joint2'] = self.rad_per_secn_rpm(
                    msg.velocities[1]) * self.joints_ratio.get('joint2')
                _current_speed_dic['joint3'] = self.rad_per_secn_rpm(
                    msg.velocities[2]) * self.joints_ratio.get('joint3')
                rospy.logwarn('msg V {} ,current V {}'.format(
                    msg.velocities, _current_speed_dic))
                # joint1 
                if _current_speed_dic.get('joint1') == 0:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint1 = STOP_DIRECTION
                elif _current_speed_dic.get('joint1') < 0:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint1 = DOWN_DIRECTION
                else:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint1 = UP_DIRECTION

                self.manipulator_joy_commander.manipulator_protocol.angularspeed.joint1 = abs(
                    _current_speed_dic.get('joint1'))
                # joint2
                if _current_speed_dic.get('joint2') == 0:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint2 = STOP_DIRECTION
                elif _current_speed_dic.get('joint2') > 0:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint2 = UP_DIRECTION
                else:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint2 = DOWN_DIRECTION

                self.manipulator_joy_commander.manipulator_protocol.angularspeed.joint2 = abs(
                    _current_speed_dic.get('joint2'))
                # joint3
                if _current_speed_dic.get('joint3') == 0:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint3 = STOP_DIRECTION
                elif _current_speed_dic.get('joint3') > 0:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint3 = DOWN_DIRECTION
                else:
                    self.manipulator_joy_commander.manipulator_protocol.direction.joint3 = UP_DIRECTION

                self.manipulator_joy_commander.manipulator_protocol.angularspeed.joint3 = abs(
                    _current_speed_dic.get('joint3'))
                # _sleep_time = (msg.time_from_start.secs + msg.time_from_start.nsecs * 1e-9) - (msg_list.points[index - 1].time_from_start.secs + msg_list.points[index - 1].time_from_start.nsecs * 1e-9 )
                # a = rospy.Duration(secs=_sleep_time)
                # rospy.sleep(a)
                self.display_data()
                # self.stop_semi_joints()
                # rospy.logwarn('sleep time is {},time from start {}'.format(_sleep_time,_sleep_time,msg.time_from_start)) 
    def display_data(self):
        rospy.logerr('1 {}, 2 {},3 {}  '.format(self.manipulator_joy_commander.manipulator_protocol.angularspeed.joint1,
                                                self.manipulator_joy_commander.manipulator_protocol.angularspeed.joint2, self.manipulator_joy_commander.manipulator_protocol.angularspeed.joint3))

    @staticmethod
    def rad_per_secn_rpm(value):
        return value*60 / (pi * 2)

    def spin(self):
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            self.manipulator_joy_commander.manipulator_protocol.send_data_old_main_board()
            self.manipulator_joy_commander.manipulator_protocol.send_sensor_board_manipulator_data()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node(node_name, anonymous=True)
    manipulator = Manipulator()
    try:
        # rospy.spin()
        manipulator.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('keyboard interrupt')
    except rospy.ROSException as err:
        logger.log_error("{}".format(err))
