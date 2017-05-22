# wm_objects_selection
This repository contains the objects selection server and client nodes

# User Manual
1 - Add the nodes in your launch files.
2 - You must call the nodes in the following order along your detection pipeline of ORK
	a. Start detection server with chosen pipeline (here linemo) : rosrun object_recognition_ros server -c `rospack find object_recognition_linemod`/conf/detection.ros.ork

	b. Start object selection server : rosrun wm_object_selection wm_obj_slct_srv.py
	c. Call object selection client : rosrun wm_object_selection wm_obj_slct_client.py
	d. Call detection clien : rosrun object_recognition_ros clientt 
3 - Publish the objet you want to retrieve on the topic /WMObjectProcessor/ObjectFilterSelection
		 
