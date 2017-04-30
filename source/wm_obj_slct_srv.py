#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import shape_msgs.msg
from object_recognition_msgs.msg import *
from object_recognition_core import *
import std_msgs.msg
from wm_object_selection.srv import *

sub_once = None


def handle_slt_obj(req):
    print "This is ",req.filter, "'s position :",req.objectarray.objects[0].pose
    return  req.objectarray.objects[0]


def slct_obj_srv():
    rospy.init_node('select_object_server')
    s = rospy.Service('slct_obj', rcgnzd_obj , handle_slt_obj)
    rospy.logout("Service select_object_server called")
    rospy.spin()


if __name__ == "__main__":
    slct_obj_srv()
