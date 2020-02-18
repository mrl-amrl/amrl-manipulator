#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryFeedback
from mercury import logger
node_name = 'arka_manipulator_controller'
# pub_topic = "/feedback_states"
class ManipulatorController :
    def __init__(self,topic_in):
        pub_topic = rospy.get_param(node_name + '/pub_topic','/feedback_states')
        rospy.Subscriber(topic_in,JointState,self._callBack)
        self.joint_trajectory_pub = rospy.Publisher(pub_topic,FollowJointTrajectoryFeedback,queue_size=1)
        self.follow_joint_trajectory_feedback_ = FollowJointTrajectoryFeedback()
    
    def _callBack(self,manipulator_states):
    
        self.follow_joint_trajectory_feedback_.joint_names = manipulator_states.name[:5]
        self.follow_joint_trajectory_feedback_.actual.positions = manipulator_states.position[:5]
        self.follow_joint_trajectory_feedback_.actual.velocities = manipulator_states.velocity[:5]
        self.follow_joint_trajectory_feedback_.actual.time_from_start = rospy.Duration(0,0)
        self.follow_joint_trajectory_feedback_.desired.time_from_start = rospy.Duration(0,0)
        self.follow_joint_trajectory_feedback_.header.stamp = rospy.Time.now()
        self.joint_trajectory_pub.publish(self.follow_joint_trajectory_feedback_)

    @staticmethod
    def spinner():
        rospy.spin()

def main():
    topic_in = rospy.get_param(node_name + '/topic_in','/joint_states')
    try:
        manipulator_controller = ManipulatorController(topic_in)
        manipulator_controller.spinner()
        rospy.signal_shutdown(node_name + ' node is shoting down')
    except rospy.ROSInternalException as e:
        logger.log_error(e + "  program interrupted before completion")
    
if __name__ == "__main__":
    rospy.init_node(node_name)
    main()
        