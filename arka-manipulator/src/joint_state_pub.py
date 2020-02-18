#!/usr/bin/env python
import os
from math import pi

import rospy
import yaml
from mercury import logger
from sensor_msgs.msg import JointState
from mercury_common.msg import ManipulatorStatus
from mercury_common.srv import SetEnabled
import subprocess


node_name = 'manipulator'


class JointStatePublisher:
    def __init__(self):
        self.path_to_read_arm_calibration = ''
        self.init_params()
        self.manual_calibration_dic = self.load_yaml_file()
        self.manual_calibrated_dic = {}
        self.init_sub_pub()
        self.elec_position = {}
        rospy.Service('/mercury/manipulator/manual_calibration',
                      SetEnabled, self.manual_calibration_cb)

    def init_params(self):
        out = subprocess.Popen(['rospack','find','arka_manipulator'],
                           stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT)
        base_path, _ = out.communicate()
        config_path = os.path.join(str(base_path[:-1]),'config/manual_calibration.yaml')
        
        if not os.path.exists(config_path):
            logger.log_error('manual calibration file {} not exist please check the path'.format(config_path))
        self.path_to_read_arm_calibration = rospy.get_param(
            '~arm_calibration_path', config_path)

    def init_sub_pub(self):
        
        rospy.Subscriber('/feedback/manipulator',
                         ManipulatorStatus, self.manipulator_states_cb)

        self.joints_pub = rospy.Publisher(
            '/joint_states', JointState, queue_size=1)

    def manipulator_states_cb(self, msg):

        self.elec_position = {'joint1_elec': msg.position[0], 'joint2_elec': msg.position[1],
                              'joint3_elec': msg.position[2], 'joint4_elec': msg.position[3]}
        self.manual_calibrated_dic['joint1_calibrated'] = self.elec_position.get(
            'joint1_elec') - self.manual_calibration_dic.get('joint1_calibration')
        self.manual_calibrated_dic['joint2_calibrated'] = self.elec_position.get(
            'joint2_elec') - self.manual_calibration_dic.get('joint2_calibration')
        self.manual_calibrated_dic['joint3_calibrated'] = self.elec_position.get(
            'joint3_elec') - self.manual_calibration_dic.get('joint3_calibration')
        self.manual_calibrated_dic['joint4_calibrated'] = self.elec_position.get(
            'joint4_elec') - self.manual_calibration_dic.get('joint4_calibration')
        self.joints_states_publisher()

    def load_yaml_file(self):
        with open(self.path_to_read_arm_calibration, 'r') as stream:
            try:
                manual_calibration_dic = yaml.safe_load(stream)
                print(manual_calibration_dic)
            except yaml.YAMLError as exc:
                print('errr ', exc)
        return manual_calibration_dic

    def write_yaml_file(self):
        with open(self.path_to_read_arm_calibration, 'w') as output:
            try:
                yaml.dump(self.manual_calibration_dic,
                          output, default_flow_style=False)
                rospy.logwarn('ARM calibrate value save')
            except yaml.YAMLError as exc:
                print('err', exc)

    @staticmethod
    def degree_to_radian(degree):
        return degree * pi / 180

    def manual_calibration_cb(self, req):
        if req.enabled:
            for i in range(1, 4):
                self.manual_calibration_dic['joint{}_calibration'.format(
                    i)] = self.elec_position.get('joint{}_elec'.format(i))
            self.write_yaml_file()
        return req.enabled

    def joints_states_publisher(self):
        joint_msg = JointState()
        joint_msg.header.frame_id = 'base_link'
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name.extend(['arka_joint1', 'arka_joint2', 'arka_joint3',
                               'arka_joint4', 'arka_joint5', 'arka_finger1', 'arka_finger2'])

        joint_msg.position.append(self.degree_to_radian(
            self.manual_calibrated_dic.get('joint1_calibrated')))
        joint_msg.position.append(self.degree_to_radian(
            self.manual_calibrated_dic.get('joint2_calibrated')))
        joint_msg.position.append(self.degree_to_radian(
            self.manual_calibrated_dic.get('joint3_calibrated')*(-1)))

        # fake_position
        joint_msg.position.extend([0, 0, 0, 0])
        self.joints_pub.publish(joint_msg)
