#!/usr/bin/env python
import rospy
import os
import sys
import object_recognition_msgs
import object_recognition_core
import object_recognition_ros
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from object_recognition_msgs.msg import *
from wm_object_selection.srv import *
from object_recognition_ros import *


class SelectObjectClient():
    def __init__(self):
        rospy.init_node('fetch_objects_array_and_name')

        rospy.logout("Fetch client initialized")
        # Retrieve name of object to filter on right topic
        rospy.logout("Wait for requested object name")
        self.ObjectFilterMessage = rospy.wait_for_message("/WMObjectProcessor/ObjectFilterSelection", String, 30)
        self.ObjectFilter = self.ObjectFilterMessage.data
        print self.ObjectFilter
        try:
            rospy.logout("Object to find received")
        except rospy.ROSException, e:
            rospy.logout("Time out")
        # Retrieve the array of detected objects from ORK server
        rospy.logout("Wait for array of recognized objects")
        self.arraytopic = rospy.wait_for_message("/recognized_object_array", RecognizedObjectArray, 30)
        try:
            rospy.logout("Objects array received")
        except rospy.ROSException, e:
            rospy.logout("Time out")

        # Retrieve point clouds of detected objects to send to grasper calculator
        rospy.logout("Wait for point clouds of")
        self.pcl_ref = rospy.wait_for_message("/real_icpin_ref", PointCloud2, 30)
        try:
            rospy.logout("Objects Point clouds received")
        except rospy.ROSException, e:
            rospy.logout("Time out")

        # Publish detected objects' point cloud to grasper input
        rospy.logout("Publish point cloud to grasper input /WMObjectProcessor/DetectedObjects")
        self.pub_pcl = rospy.Publisher("/WMObjectProcessor/DetectedObjects", PointCloud2,
                                  queue_size=100)
        self.PubObjectPose = rospy.Publisher("/WMObjectProcessor/SlctObjectPose", PoseWithCovarianceStamped,
                                             queue_size =100)

    def client(self, array, pcl_ref, obj_filter):
        try:
            print "I want ", obj_filter, " in ", self.arraytopic

        except rospy.ROSException, e:
            print "Nothing published, try again: %s" % e
        rospy.wait_for_service('slct_obj')
        try:
            self.inputs_srv = rospy.ServiceProxy('slct_obj', rcgnzd_obj)
            self.slct_obj = self.inputs_srv(array, pcl_ref, obj_filter)
            print self.slct_obj.SlctObjectPose
            self.PubObjectPose.publish(self.slct_obj.SlctObjectPose)
            self.pub_pcl.publish(self.pcl_ref)

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e



if __name__ == "__main__":
    SelectObjectClient = SelectObjectClient()
    SelectObjectClient.client(SelectObjectClient.arraytopic, SelectObjectClient.pcl_ref, SelectObjectClient.ObjectFilter)

