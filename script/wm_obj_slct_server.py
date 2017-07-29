#!/usr/bin/env python
# coding=utf-8
import rospy
import time
import os
import cv2
import random
from object_recognition_msgs.srv import *
from cv_bridge import CvBridge, CvBridgeError
from wm_objects_selector.srv import *
from visualization_msgs.msg import Marker


def handle_slt_obj(req):
    objects_info_list = []
    i_p = 0
    rospy.logdebug("Request for ", req.filter, "registered")
    rospy.logout('Retrieving object information')

    # Prepare directory to save images of recognition
    path = os.path.expanduser('~/roi_images_objects')
    if os.path.exists(path):
        os.system('rm -r ' + path)
    os.mkdir(path)
    # Build list of objects' information
    for i in req.objectarray.objects:
        objects_info_list.append(client_info(i.type))
        image = req.image
        image_msg = bounding_box(image,
                                  i.pose.pose.pose.position,
                                  objects_info_list[-1].name,
                                  objects_info_list[-1].ground_truth_mesh,
                                  path)

    # Change req.filter to random object if filter is 'random'
    if req.filter == 'unknown':
        req.filter = objects_info_list[0].name  # Best confidence is first in the list

    # Find the requested object in the list
    for i in objects_info_list:
        if i.name == req.filter:
            object_key = req.objectarray.objects[i_p].type.key
            object_pos = req.objectarray.objects[i_p].pose
            rospy.logdebug('Object found: The object with id'
                           + str(object_key)
                           + 'is :'
                           + str(i.name)
                           + '\n\rat position\r\n'
                           + str(object_pos))
            rospy.logout("Service select_object_server waiting")
            grab_workspace = set_grap_workspace(object_pos.pose.pose.position, i.ground_truth_mesh)
            # Build response
            resp = RecognizeObjectResponse()
            resp.seg_image = image_msg
            # Transform object pose to base frame
            resp.selected_object_pose = object_pos
            resp.workspace = grab_workspace
            return resp
        else:
            i_p = i_p + 1
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


def bounding_box(image, position, name, mesh, path):
    bridge = CvBridge()
    x_val = []
    y_val = []
    z_val = []
    x_min = 0
    x_max = 0
    y_min = 0
    y_max = 0

    if os.path.exists(path+'/scene.png'):
        cv_image = cv2.imread(path+'/scene.png')
        cv_image_sep = bridge.imgmsg_to_cv2(image, "passthrough")
        b, g, r = cv2.split(cv_image_sep)  # get b,g,r
        cv_image_sep = cv2.merge([r, g, b])  # switch to rgb
    else:
        try:
            cv_image = bridge.imgmsg_to_cv2(image, "passthrough")
            cv_image_sep = cv_image
            b, g, r = cv2.split(cv_image)  # get b,g,r
            cv_image = cv2.merge([r, g, b])  # switch to rgb
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
        y_val.append(i.y)
        z_val.append(i.z)
    z_max = max([max(z_val), max(y_val), max(x_val)]) * 1000

    # Transfer object size to pixel frame
    size_v = int(z_max / z_r * 1.5)
    size_u = int(z_max / z_r * 1.5)

    # Randomize color of rectangle
    component = lambda: random.randint(0, 255)
    color = (component(), component(), component())

    # Draw bounding box cv_image
    cv2.rectangle(cv_image, (x_p-size_u/2, y_p-size_v/2), (x_p+size_u/2, y_p+size_v/2), color, 2)
    cv2.putText(cv_image, name, (x_p-size_u/2, y_p-size_v/2), 0, 1, color, 2)

    # Draw bounding box cv_image_sep
    x_min = x_p - size_u / 2
    y_min = y_p - size_v / 2
    x_max = x_p + size_u / 2
    y_max = y_p + size_v / 2
    # If box is outside boundaries, resize to image
    if x_min < 0:
        x_min = 0
    if y_min < 0:
        y_min = 0
    if x_max > 640:
        x_max = 640
    if y_max > 480:
        y_max = 480

    cv2.rectangle(cv_image_sep, (x_min, y_min), (x_max, y_max), color, 2)
    cv2.putText(cv_image_sep, name, (x_p - size_u / 2, y_p - size_v / 2), 0, 1, color, 2)

    # Crops individual object
    crop_img = cv_image_sep[y_p - size_v / 2 - 20:y_p + size_v / 2, x_p - size_u / 2:x_p + size_u / 2+40]

    # Resize object to 640 pixel wide
    r = 640.0 / crop_img.shape[1]
    dim = (640, int(crop_img.shape[0] * r))

    # perform the actual resizing of the image and show it
    crop_img_resized = cv2.resize(crop_img, dim, interpolation=cv2.INTER_AREA)

    # Adjust saving path and saves images
    os.chdir(os.path.expanduser(path))
    cv2.imwrite(name + '_' + str(time.strftime("%H:%M:%S")) + '.png', crop_img_resized)
    cv2.imwrite('scene.png', cv_image)
    os.chdir(os.path.expanduser('~/'))


    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
    return image_message


def set_grap_workspace(position, mesh):
    pub_pcl = rospy.Publisher("/WMObjectProcessor/workspace", Marker,
                              queue_size=100)

    x_val = []
    y_val = []
    z_val = []

    # Extract 3d position of object
    z_r = position.z
    y_r = position.y
    x_r = position.x

    # Extract width and length of object in millimeters
    for i in mesh.vertices:
        x_val.append(i.x)
        y_val.append(i.y)
        z_val.append(i.z)
    size_max = max([max(z_val), max(y_val), max(x_val)])*1.5

    # Create workspace from position and size of object
    w_min_x = x_r - size_max / 2
    w_max_x = x_r + size_max / 2
    w_min_y = y_r - size_max / 2
    w_max_y = y_r + size_max / 2
    w_min_z = z_r - size_max / 2
    w_max_z = z_r + size_max / 2

    # Create marker for representation
    marker = Marker()
    marker.header.frame_id = "/camera_rgb_optical_frame"
    #marker.header.stamp = rospy.get_time()
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = size_max
    marker.scale.y = size_max
    marker.scale.z = size_max
    marker.color.a = 1
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x_r
    marker.pose.position.y = y_r
    marker.pose.position.z = z_r

    pub_pcl.publish(marker)
    grab_workspace = str(w_min_x) + ',' + str(w_max_x) + ',' + str(w_min_y) + ',' + str(w_max_y) + ',' + str(w_min_z) + ',' + str(w_max_z)
    return grab_workspace

if __name__ == "__main__":
    slct_obj_srv()
