#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header
import cv2
from ultralytics import YOLO
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField

class ObjectDetector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('object_detector', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Create a subscriber for the image topic
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.rgbImageCallback , queue_size=1)
        rospy.Subscriber('/xyzImg', Float32MultiArray, self.matrix_cb, queue_size=1)
        self.picking_pub = rospy.Publisher('picking_pcl', PointCloud2, queue_size=1)
        self.target_pub = rospy.Publisher('target_pcl', PointCloud2, queue_size=1)

        # Load the YOLO model (YOLOv5 or YOLOv8)
        self.model = YOLO('best.pt')  # You can replace with 'yolov8s.pt' for YOLOv8

        # Variable to hold the latest image
        self.latest_image = None
        self.connector_annotated_frame = None
        self.target_annotated_frame = None
        self.main_loop()
        # Run the main loop
        rospy.spin()


    def matrix_cb(self, msg):
        mat = np.array(msg.data)
        mat = mat.reshape(msg.layout.dim[1].size, msg.layout.dim[0].size, msg.layout.dim[2].size)
        self.xyzImg = mat

    def rgbImageCallback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def main_loop(self):
        # Main loop to handle keyboard input

        while not rospy.is_shutdown():
            try:
                self.connector = self.latest_image[130:350, 500:750]
                self.target = self.latest_image[350:900, 500:1000]
                # self.latest_image = self.latest_image[200:720, 400:1100]

                # Run object detection
                # results = self.model.predict(self.latest_image, conf=0.5, iou=0.5)
                connector_result = self.model.predict(self.connector, conf=0.5, iou=0.5, max_det=1)
                target_result = self.model.predict(self.target, conf=0.5, iou=0.5, max_det=3)
                # connector_result = self.model(self.target)

                # Draw bounding boxes on the image

                self.connector_annotated_frame = connector_result[0].plot()
                self.connector_boxes = connector_result[0].boxes

                self.target_annotated_frame = target_result[0].plot()
                self.target_boxes = target_result[0].boxes

            except Exception as e:
                rospy.logerr("Error processing image: %s", str(e))

            if self.connector_annotated_frame is not None:
                if self.connector_boxes.xywh.numel() != 0 :
                    x, y, w, h = np.floor(self.connector_boxes.xyxy[0].cpu().numpy())
                    x = int(x)
                    y = int(y)
                    w = int(w)
                    h = int(h)
                    point_x = int((y+h)/2) + 130
                    point_y = int((x+w)/2) + 500
                    image = self.connector_annotated_frame[y:h, x:w]
                    #image = self.connector_annotated_frame
                    mask = (image[:, :, 0] >= 100) & (image[:, :, 1] >= 100) & (image[:, :, 2] >= 100) & (image[:, :, 0] <= 200) & (image[:, :, 1] <= 200) & (image[:, :, 2] <= 200)
                    image[mask] = [0, 0, 0]
                    mask_x_set, mask_y_set = np.where(mask == True)
                    mask_x = int(np.mean(mask_x_set))
                    mask_y = int(np.mean(mask_y_set))

                    print(mask_x, mask_y)



                    fields = [PointField('x', 0, PointField.FLOAT32, 1),
                              PointField('y', 4, PointField.FLOAT32, 1),
                              PointField('z', 8, PointField.FLOAT32, 1),
                              PointField('rgb', 12, PointField.UINT32, 1)]
                    header = Header()
                    header.frame_id = "map"
                    header.stamp = rospy.Time.now()
                    points_x, points_y, points_z = self.xyzImg[mask_x + 130 + y, mask_y + 500 + x]
                    my_points = []
                    my_points.append([points_x, points_y, points_z, 215])
                    pc2 = point_cloud2.create_cloud(header, fields, my_points)
                    self.picking_pub.publish(pc2)
                    cv2.imshow('Connector', image)
                    cv2.waitKey(1)  # Refresh window
            else:
                rospy.logwarn("Failed to annotate image.")

            if self.target_annotated_frame is not None:
                if self.target_boxes.xywh.numel() != 0 :

                    header = Header()
                    header.frame_id = "map"
                    header.stamp = rospy.Time.now()
                    fields = [PointField('x', 0, PointField.FLOAT32, 1),
                              PointField('y', 4, PointField.FLOAT32, 1),
                              PointField('z', 8, PointField.FLOAT32, 1),
                              PointField('rgb', 12, PointField.UINT32, 1)]
                    my_points = []
                    for i in range(self.target_boxes.xyxy.shape[0]) :
                        x, y, w, h = np.floor(self.target_boxes.xyxy[i].cpu().numpy())
                        x = int(x)
                        y = int(y)
                        w = int(w)
                        h = int(h)
                        point_x = int((y+h)/2) + 350
                        point_y = int((x+w)/2) + 500
                        points_x, points_y, points_z = self.xyzImg[point_x, point_y]
                        my_points.append([points_x, points_y, points_z, 255])

                    #image = self.target_annotated_frame[y:h, x:w]
                    image = self.target_annotated_frame


                    pc2 = point_cloud2.create_cloud(header, fields, my_points)
                    self.target_pub.publish(pc2)

                    cv2.imshow('Target', image)
                    cv2.waitKey(1)  # Refresh window

            else:
                rospy.logwarn("Failed to annotate image.")
        # Cleanup
        cv2.destroyAllWindows()



if __name__ == '__main__':
    try:
        ObjectDetector()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
