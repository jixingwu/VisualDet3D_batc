#!/bin/bash
rosbag record 
/visualDet3D/bboxes \\
/visualDet3D/cube \\
/visualDet3D/cube_array \\
/visualDet3D/depth \\
/visualDet3D/image \\ 
/visualDet3D/odometry \\
 -O datatopics.bag


