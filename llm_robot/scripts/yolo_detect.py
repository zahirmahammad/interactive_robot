from ultralytics import YOLO
import rospy
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
import cv2

class YoloDetector:
    def __init__(self):
        # Initialize the node
        rospy.init_node('yolo_detector')
        self.rgb_image = None

        # Initialize the CV bridge
        self.bridge = CvBridge()

        # Load the YOLO model
        self.model = YOLO('yolo11m.pt')  # Load the YOLOv8 model
        self.model.fuse()  # Fuse the model for faster inference
        rospy.loginfo("YOLO detector initialized. Waiting for messages...")


        # Subscribe to the camera image topic
        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)
        rospy.loginfo("Subscribed to camera image topic.")

    
    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # print(self.rgb_image.shape)
        detected_image = self.detect_objects(self.rgb_image)
        # Display the detected image
        cv2.imshow("YOLO Detection", detected_image)
        cv2.waitKey(1)

    
    def detect_objects(self, image):
        # Perform object detection
        results = self.model(image, conf=0.8)
        # Process the results
        for result in results:
            boxes = result.boxes.xyxy
            confidences = result.boxes.conf
            classes = result.boxes.cls
            for box, conf, cls in zip(boxes, confidences, classes):
                x1, y1, x2, y2 = map(int, box)
                label = f"{self.model.names[int(cls)]} {conf:.2f}"
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return image


if __name__ == '__main__':
    yolo_detector = YoloDetector()
    rospy.spin()
