#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2

class PixelTo3DConverter:
    def __init__(self):
        rospy.init_node('pixel_to_3d_converter')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Camera parameters (will be updated from camera_info)
        self.camera_info = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # Latest RGB and depth images
        self.rgb_image = None
        self.depth_image = None
        
        # Subscribe to RGB, depth, and camera info topics
        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.camera_info_callback)
        
        rospy.loginfo("Pixel to 3D converter initialized. Waiting for messages...")
        
    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting RGB image: {e}")
            
    def depth_callback(self, msg):
        try:
            # Convert depth image - note that depth encoding might vary
            # Common encodings: 16UC1, 32FC1
            if msg.encoding == '16UC1':
                # Convert from millimeters to meters
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough") / 1000.0
            elif msg.encoding == '32FC1':
                # Already in meters
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            else:
                rospy.logwarn(f"Unexpected depth encoding: {msg.encoding}")
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {e}")
    
    def camera_info_callback(self, msg):
        self.camera_info = msg
        # Extract camera intrinsic parameters
        self.fx = msg.K[0]  # Focal length x
        self.fy = msg.K[4]  # Focal length y
        self.cx = msg.K[2]  # Principal point x
        self.cy = msg.K[5]  # Principal point y
        
        # Unsubscribe after receiving camera info
        self.camera_info_sub.unregister()
        rospy.loginfo("Camera info received. Ready to convert pixels to 3D points.")
    
    def pixel_to_3d(self, u, v):
        """
        Convert pixel coordinates (u, v) to 3D point (X, Y, Z) in camera frame
        """
        if self.rgb_image is None or self.depth_image is None:
            rospy.logwarn("RGB or depth image not available yet")
            return None
            
        if self.camera_info is None:
            rospy.logwarn("Camera info not available yet")
            return None
            
        # Check if pixel is within image bounds
        if u < 0 or u >= self.depth_image.shape[1] or v < 0 or v >= self.depth_image.shape[0]:
            rospy.logwarn(f"Pixel ({u}, {v}) is outside image bounds")
            return None
            
        # Get depth value at pixel (u, v)
        Z = self.depth_image[v, u]
        
        # Check for invalid depth values
        if np.isnan(Z) or Z <= 0:
            rospy.logwarn(f"Invalid depth value at pixel ({u}, {v})")
            return None
            
        # Convert from pixel coordinates to 3D coordinates
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        
        return (X, Y, Z)
    
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.rgb_image is not None and self.depth_image is not None and self.camera_info is not None:
                # Example: Get 3D coordinates for center pixel
                center_u = self.rgb_image.shape[1] // 2
                center_v = self.rgb_image.shape[0] // 2
                
                point_3d = self.pixel_to_3d(center_u, center_v)
                if point_3d:
                    rospy.loginfo(f"3D coordinates at pixel ({center_u}, {center_v}): X={point_3d[0]:.3f}, Y={point_3d[1]:.3f}, Z={point_3d[2]:.3f} meters")
                
                # You can also get 3D coordinates for any other pixel of interest
                # For example, to get coordinates for a specific pixel (100, 200):
                # point_3d = self.pixel_to_3d(100, 200)
                
            rate.sleep()

if __name__ == '__main__':
    try:
        converter = PixelTo3DConverter()
        converter.run()
    except rospy.ROSInterruptException:
        pass
