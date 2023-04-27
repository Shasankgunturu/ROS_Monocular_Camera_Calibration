#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from camera_info_manager import CameraInfoManager

def main():
    rospy.init_node('my_camera_node')
    bridge = CvBridge()
    # Create a publisher for the camera data
    image_publisher = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

    # Create a camera info manager to handle camera_info
    camera_info_manager = CameraInfoManager(cname='my_camera_node', url='file:///home/shasankgunturu/camera_info.yaml', namespace='camera/')
    camera_info_manager.loadCameraInfo()

    # Create a publisher for the camera info
    camera_info_publisher = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)

    # Create a rate object to control the publishing rate
    rate = rospy.Rate(10)
    cap = cv2.VideoCapture(2)

    while not rospy.is_shutdown():
        # Capture camera data and publish it
        ret, frame = cap.read()
        if ret:
            # Convert the OpenCV image to a ROS image message
            img_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            # Publish the ROS image message
            image_publisher.publish(img_msg)
                    # Publish camera info
            camera_info = camera_info_manager.getCameraInfo()
            camera_info.header.stamp = rospy.Time.now()
            camera_info_publisher.publish(camera_info)

    # Release the OpenCV capture and shutdown the node
    cap.release()
    rospy.shutdown()

        # Sleep for a short time to control the publishing rate

if __name__ == '__main__':
    main()
