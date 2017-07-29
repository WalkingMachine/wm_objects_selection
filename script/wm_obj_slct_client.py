#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from object_recognition_msgs.msg import *
from wm_objects_selector.srv import *

# This class initialize the client to subscrible and publish to the right topics
# It waits for all right messages before being ready to be called


class SelectObjectClient:
    def __init__(self):
        # Initialize node
        rospy.init_node('fetch_objects_array_and_name')
        rospy.logout("Fetch client initialized")

        # Retrieve name of object to filter on right topic
        rospy.logout("Wait for requested object name")
        self.object_filter_message = rospy.wait_for_message("/WMObjectProcessor/ObjectFilterSelection", String, 30)
        self.object_filter = self.object_filter_message.data
        try:
            rospy.logout("Object to find received")
        except rospy.ROSException, e:
            rospy.logout("Time out")

        # Retrieve point clouds of detected objects to send to grasper calculator
        rospy.logout("Wait for point clouds of segmented objects")
        self.pcl_ref = rospy.wait_for_message("/real_icpin_ref", PointCloud2, 30)
        try:
            rospy.logout("Objects Point clouds received")
        except rospy.ROSException, e:
            rospy.logout("Time out")

        # Retrieve image to add bounding boxes to it
        rospy.logout("Wait for image")
        self.image_msg = rospy.wait_for_message("/base_xtion/rgb/image_rect_color", Image, 30) # for kinect2 : '/kinect2/qhd/image_color_rect' for xtion : '/camera/rgb/image_rect_color'
        try:
            rospy.logout("Image received")
        except rospy.ROSException, e:
            rospy.logout("Time out")

        # Retrieve the array of detected objects from ORK server
        rospy.logout("Wait for array of recognized objects")
        self.arraytopic = rospy.wait_for_message("/recognized_object_array", RecognizedObjectArray, 30)
        try:
            rospy.logout("Objects array received")
        except rospy.ROSException, e:
            rospy.logout("Time out")

        # Publish detected objects' point cloud to grasper input
        rospy.logout("Publish point cloud to grasper input /WMObjectProcessor/DetectedObjects")
        self.pub_pcl = rospy.Publisher("/WMObjectProcessor/DetectedObjects", PointCloud2,
                                  queue_size=100)
        self.PubObjectPose = rospy.Publisher("/WMObjectProcessor/selected_object_pose", PoseWithCovarianceStamped,
                                             queue_size=100)

    # Method to call the server once all required information is received
    def client(self, array, pcl_ref, obj_filter, image):
        rospy.logout("Requesting position of " + obj_filter)
        rospy.wait_for_service('slct_obj')
        try:
            self.inputs_srv = rospy.ServiceProxy('slct_obj', RecognizeObject)
            self.slct_obj = self.inputs_srv(array, pcl_ref, obj_filter, image)
            rospy.logout('Service found the requested object at\n\r ' + str(self.slct_obj.selected_object_pose))
            self.PubObjectPose.publish(self.slct_obj.selected_object_pose)
            self.pub_pcl.publish(self.pcl_ref)

        except rospy.ServiceException, e:
            if str(e).find('list index out of range'):
                rospy.logout('Service did not find the requested object')
            else:
                rospy.logerr('Service call failed: ' + str(e))


if __name__ == "__main__":
    SelectObjectClient = SelectObjectClient()
    SelectObjectClient.client(SelectObjectClient.arraytopic,
                              SelectObjectClient.pcl_ref,
                              SelectObjectClient.object_filter,
                              SelectObjectClient.image_msg)


