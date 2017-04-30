#!/usr/bin/env python
# coding=utf-8
import rospy
import os
import object_recognition_core
import geometry_msgs.msg
import sensor_msgs.msg
import shape_msgs.msg
from object_recognition_msgs.msg import *
from object_recognition_msgs.srv import *
from object_recognition_core import *
import std_msgs.msg
from wm_object_selection.srv import *


def handle_slt_obj(req):
    objects_info_list = []
    position = 0

    print "Request for ", req.filter, "registered"
    # Construit la liste des info d'objets détectés
    for i in req.objectarray.objects:
        objects_info_list.append(client_info(i.type))

    # Parcours la liste d'informations pour trouver l'objet demandé
    for i in objects_info_list:
        if i.name == req.filter:
            print "Object found: The object with id",req.objectarray.objects[position].type.key, "is :",i.name
            break
        else:
            rospy.logout("Object not found, try again")
        position = position +1


    rospy.logout("Service select_object_server waiting")
    return  req.objectarray.objects[0]


def slct_obj_srv():
    rospy.init_node('select_object_server')
    s = rospy.Service('slct_obj', rcgnzd_obj , handle_slt_obj)
    rospy.logout("Service select_object_server waiting")
    os.system("rosrun object_recognition_ros object_information_server")
    rospy.spin()

def client_info(type):
    rospy.wait_for_service('get_object_info')
    print "Retrieving object info"
    try:
        h_obj_info = rospy.ServiceProxy('get_object_info', GetObjectInformation)
        info = h_obj_info(type)
        return info.information

    except rospy.ServiceException, e:
        print "Service get_object_info failed: %s" % e


if __name__ == "__main__":
    slct_obj_srv()
