#!/usr/bin/python

import message_filters
from sensor_msgs.msg import PointCloud2, Image
from jeff_segment_objects.srv import SegmentObjects
from cv_bridge import CvBridge, CvBridgeError

import rospy
import cv2

def callback(cloud_msg, image_msg):
    print 'callback!'
    rospy.wait_for_service('segment_objects')
    try:
        segment_objects = rospy.ServiceProxy('segment_objects', SegmentObjects)
        res = segment_objects(cloud_msg, 0.2, 0.035)
        
        # get the cv2 image
        try:
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except CvBridgeError:
            return

        width = cloud_msg.width
        height = cloud_msg.height

        # for each cluster
        for cluster in res.clusters:
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
            cv2.imshow('image', frame)
            cv2.waitKey(1)
    
    except rospy.ServiceException as e:
        print e

rospy.init_node('get_segmented_objects')
cloud_sub = message_filters.Subscriber('/xtion/depth_registered/points', PointCloud2)
image_sub = message_filters.Subscriber('/xtion/rgb/image_rect_color', Image)
ts = message_filters.ApproximateTimeSynchronizer([cloud_sub, image_sub], 1, 0.1)
ts.registerCallback(callback)
rospy.spin()