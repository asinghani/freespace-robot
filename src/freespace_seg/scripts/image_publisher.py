import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImagePublisher:
    def __init__(self, pub_topic, half_res=False):
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(pub_topic, Image, queue_size=1)

        self.half_res = half_res

    def publish(self, image, rgb=False):
        """
        Publishes an OpenCV format image to the ROS topic
        """
        if self.half_res:
            image = image[::2, ::2, :]

        image = image.astype(np.uint8)

        try:
            if rgb:
                self.publisher.publish(self.bridge.cv2_to_imgmsg(image, "rgb8"))
            else:
                self.publisher.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except Exception as e:
            rospy.logerr("Error while publishing image: {}".format(e))
