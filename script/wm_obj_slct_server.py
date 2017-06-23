#!/usr/bin/env python
# coding=utf-8
import rospy
import os
import cv2
import object_recognition_core
import geometry_msgs.msg
import shape_msgs.msg
import std_msgs.msg
from object_recognition_msgs.msg import *
from object_recognition_msgs.srv import *
from object_recognition_core import *
from cv_bridge import CvBridge, CvBridgeError
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
        bounding_box(req.image,
                     i.pose.pose.pose.position,
                     objects_info_list[-1].name,
                     objects_info_list[-1].ground_truth_mesh)

    # Parcours la liste d'informations pour trouver l'objet demandé
    for i in objects_info_list:
        if i.name == req.filter:
            rospy.logdebug('Object found: The object with id'
                           + str(req.objectarray.objects[position].type.key)
                           + 'is :'
                           + str(i.name)
                           + '\n\rat position\r\n'
                           + str(req.objectarray.objects[position].pose.pose.pose.position))
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
        rospy.logerr("Service get_object_info failed: %s" + str(e))


def bounding_box(image, position, name, mesh):
    bridge = CvBridge()
    x_val = []
    z_val = []
    try:
        cv_image = bridge.imgmsg_to_cv2(image, "passthrough")
    except CvBridgeError as e:
        print(e)
    print name

    f = 0.00966
    fx = 525.0  # default focal length x
    fy = 525.0  # default focal length y
    cx = 319.5  # default optical center x
    cy = 239.5  # default optical center y

    # Extract 3d position of object
    z_r = position.z
    y_r = position.y
    x_r = position.x

    # Transfer to pixel frame
    d = z_r / f
    u = x_r * fx / z_r + cx
    v = y_r * fy / z_r + cy
    x_p = int(u) - 20
    y_p = int(v)

    # Extract width and length of object in millimeters
    for i in mesh.vertices:
        x_val.append(i.x)
        z_val.append(i.z)
    x_max = max(x_val) * 1000
    z_max = max(z_val) * 1000

    # Transfer object size to pixel frame
    size_v = int(x_max / z_r * 2)
    size_u = int(z_max / z_r * 2)

    # Draw bounding box
    cv2.rectangle(cv_image, (x_p-size_u/2, y_p-size_v/2), (x_p+size_u/2, y_p+size_v/2), (255, 0, 0), 2)
    cv2.putText(cv_image, name, (x_p-size_u/2, y_p-size_v/2), 0, 1, 0xFF, 2)

    # Crops individual object
    crop_img = cv_image[y_p - size_v / 2 - 20:y_p + size_v / 2, x_p - size_u / 2:x_p + size_u / 2]

    # Resize object
    r = 640.0 / crop_img.shape[1]
    dim = (640, int(crop_img.shape[0] * r))

    # perform the actual resizing of the image and show it
    crop_img_resized = cv2.resize(crop_img, dim, interpolation=cv2.INTER_AREA)

    # Saves different images to directory
    path = os.path.expanduser('~/roi_images_objects')
    if(os.path.exists(path)):
        os.system('rm -r ' + path)
    os.system('mkdir roi_images_objects')
    os.chdir(os.path.expanduser('~/roi_images_objects/'))
    cv2.imwrite(name + '.png', crop_img_resized)
    cv2.imwrite('scene.png', cv_image)
    os.chdir(os.path.expanduser('~/'))


if __name__ == "__main__":
    slct_obj_srv()