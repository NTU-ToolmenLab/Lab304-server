#!/usr/bin/env python
# -*- coding: utf-8 -*-

from abc import ABCMeta, abstractmethod
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import requests

__author__ = "Sam Kuo"
__maintainer__ = "Sam Kuo"
__email__ = "kuo77122@gmail.com"
__version__ = "0.1"


class RTLS(object):

    def __init__(self, topic, server_url, frame_id, **kwargs):
        self.frame_id = frame_id
        self.pose_pub = rospy.Publisher(topic, PoseWithCovarianceStamped, queue_size=10)
        self.server_url = server_url
        try:
            response = requests.get(self.server_url)
            response.raise_for_status()
        except Exception as e:
            rospy.logfatal("Cannot build connection to server {}. {}".format(self.server_url, e))

    def decode_msg(self, raw_msg):
        pose_stamped = PoseWithCovarianceStamped()
        pose_stamped.header.frame_id = self.frame_id
        pose_stamped.header.stamp = rospy.Time.now()

        pose_stamped.pose.pose.position.x = raw_msg.get("x")
        pose_stamped.pose.pose.position.y = raw_msg.get("y")
        # pose_stamped.pose.pose.position.z = position.get("z")

        # RTLS donot contain orientation, skip
        # pose_stamped.pose.pose.orientation.y = orientation.get("x")
        # pose_stamped.pose.pose.orientation.x = orientation.get("y")
        # pose_stamped.pose.pose.orientation.z = orientation.get("z")
        # pose_stamped.pose.pose.orientation.w = orientation.get("w")

        # RTLS may provide convariance in the future
        # pose_stamped.pose.covariance = covariance
        return pose_stamped

    def get_position(self):
        """
        curl http://192.168.49.3:5000/api/v1/tagPosition
        {"name":"Tag-01","x":141.01423987174698,"y":440.1414863167405
        """
        try:
            response = requests.get(self.server_url)
            response.raise_for_status()
        except Exception as e:
            rospy.logwarn("loss connection with server {}. {}".format(self.server_url, e))
        try:
            data = response.json()
            if "x" not in data.keys() or "y" not in data.keys():
                rospy.logerror("Decode Error. cannot find x, y in response")
                return
            return data
        except Exception as e:
            rospy.logerror(e)

    def send2topic(self, pose_stamped):
        self.pose_pub.publish(pose_stamped)


if __name__ == '__main__':
    rospy.init_node('rtls')
    server_url = rospy.get_param('~server_url')
    localization_topic = rospy.get_param('~pose_topic', '/rtls_pose')
    query_rate = rospy.get_param('~query_rate', 1)
    frame_id = rospy.get_param('~frame_id', 'rtls')
    rate = rospy.Rate(query_rate)

    rtls_api = RTLS(topic=localization_topic,
                    server_url=server_url,
                    frame_id=frame_id)

    while not rospy.is_shutdown():
        raw_msg = rtls_api.get_position()
        if raw_msg is not None:
            pose_stamped = rtls_api.decode_msg(raw_msg)
            rtls_api.send2topic(pose_stamped)
        rate.sleep()
