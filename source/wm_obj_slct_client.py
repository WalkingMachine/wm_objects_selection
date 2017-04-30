#!/usr/bin/env python
import rospy
import sys
from object_recognition_msgs.msg import *
import std_msgs.msg
from wm_object_selection.srv import *


class select_object_client():
    def __init__(self):
        rospy.init_node('fetch_objects_array_and_name')
        self.arraytopic = rospy.wait_for_message("/recognized_object_array", RecognizedObjectArray, 2)
        self.filter = "coke"
        print "I want ",self.filter," in ",self.arraytopic

    def client(self,array,filter):
        rospy.wait_for_service('slct_obj')
        print "Fetching object for you"
        try:
            self.inputs_srv = rospy.ServiceProxy('slct_obj', rcgnzd_obj)
            self.slct_obj = self.inputs_srv(array,filter)

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


if __name__ == "__main__":
    slct_obj_clt = select_object_client()
    slct_obj_clt.client(slct_obj_clt.arraytopic, slct_obj_clt.filter)
