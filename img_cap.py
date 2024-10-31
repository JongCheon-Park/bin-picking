#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSaver:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_saver', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Create a subscriber for the image topic
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.rgbImageCallback)

        # Variable to hold the latest image
        self.latest_image = None

        # OpenCV window setup
        cv2.namedWindow('Image Window', cv2.WINDOW_NORMAL)

        # Run the main loop
        self.save = 1
        self.main_loop()

    def rgbImageCallback(self, msg):

        # Convert ROS image message to OpenCV image
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = self.latest_image[130:350, 500:750]

        except Exception as e:
            rospy.logerr("Error converting image: %s", str(e))

    def main_loop(self):
        # Main loop to handle keyboard input
        while not rospy.is_shutdown():
            cv2.imshow('img', self.latest_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('a'):
                if self.latest_image is not None:
                    # Save the latest image to a file
                    filename = self.save
                    cv2.imwrite(f'data/image_{filename}.png', self.latest_image)
                    self.save += 1
                    rospy.loginfo("Image saved as image_%s.png", self.save)
                else:
                    rospy.logwarn("No image available to save")
            elif key == 27:  # 'ESC' key to exit
                break

        # Cleanup
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        ImageSaver()
    except rospy.ROSInterruptException:
        pass
