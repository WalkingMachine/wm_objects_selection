#!/usr/bin/env python
# coding=utf-8
import rospy
import os
import object_recognition_core
import geometry_msgs.msg
import shape_msgs.msg
from object_recognition_msgs.msg import *
from object_recognition_msgs.srv import *
from object_recognition_core import *
import std_msgs.msg
from wm_object_selection.srv import *
from sensor_msgs.msg import PointCloud2


def handle_slt_obj(req):
    objects_info_list = []
    position = 0
    rospy.logdebug("Request for ", req.filter, "registered")
    rospy.logout('Retrieving object information')
    # Construit la liste des info d'objets détectés
    for i in req.objectarray.objects:
        objects_info_list.append(client_info(i.type))

    # Parcours la liste d'informations pour trouver l'objet demandé
    for i in objects_info_list:
        if i.name == req.filter:
            rospy.logdebug('Object found: The object with id'+ str(req.objectarray.objects[position].type.key)+ 'is :'+ str(i.name)+ '\n\rat position\r\n'+ str(req.objectarray.objects[position].pose.pose.pose.position))
            rospy.logout("Service select_object_server waiting")
            return req.objectarray.objects[position].pose
        else:
            position = position + 1
    rospy.logout("Object not found")
    rospy.logout("Service select_object_server waiting")


def slct_obj_srv():
    rospy.init_node('select_object_server')
    # Create Service for object info retrieving
    s = rospy.Service('slct_obj', RecognizeObject, handle_slt_obj)
    rospy.logout("Service select_object_server waiting")
    os.system("rosrun object_recognition_ros object_information_server")
    rospy.spin()

def client_info(type):
    rospy.wait_for_service('get_object_info')
    try:
        h_obj_info = rospy.ServiceProxy('get_object_info', GetObjectInformation)
        info = h_obj_info(type)
        return info.information

    except rospy.ServiceException, e:
        rospy.logerr("Service get_object_info failed: %s" +str(e))


if __name__ == "__main__":
    slct_obj_srv()
