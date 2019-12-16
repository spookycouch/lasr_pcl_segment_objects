import rospy
import message_filters

import numpy as np
import cv2

from sensor_msgs.msg import Image, PointCloud2
from jeff_segment_objects.msg import Cluster
from jeff_segment_objects.srv import SegmentObjects
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

while not rospy.is_shutdown():
    rospy.wait_for_service('segment_objects')
    try:
        segment_objects = rospy.ServiceProxy('segment_objects', SegmentObjects)
        res = segment_objects(0.035, 0.2)

        pcl = res.cloud
        image = res.image
        clusters = res.clusters

        # get pcl
        height = pcl.height
        width = pcl.width
        point_step = pcl.point_step
        cloud = np.fromstring(pcl.data, np.float32)
        cloud = cloud.reshape(height * width, 8)

        # get the cv2 image
        try:
            frame = bridge.imgmsg_to_cv2(image, 'bgr8')
        except CvBridgeError:
            break

        # for each cluster
        print len(clusters.clusters)
        for cluster in clusters.clusters:
            max_index = cluster.indices[0]
            left = cluster.indices[0]%width
            right = cluster.indices[0]%width
            bottom = int(cluster.indices[0]/width)
            top = int(cluster.indices[0]/width)

            count = 0 
            for index in cluster.indices:
                row = int(index/width)
                col = index%width
                left = min(left, col)
                right = max(right, col)
                top = min(top, row)
                bottom = max(bottom, row)
                
                frame[row,col] = (0,255,0)
                
            cv2.rectangle(frame, (left, top), (right, bottom), (255, 255, 0), 1)
        cv2.imshow('client', frame)
        cv2.waitKey(1)
    except rospy.ServiceException as e:
        print e