import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point

def point_3d_to_2d(point_3d, from_frame, to_frame):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Create a PoseStamped message with the 3D point
    pose_3d = PoseStamped()
    pose_3d.header.frame_id = from_frame
    pose_3d.header.stamp = rospy.Time.now()
    pose_3d.pose.position = point_3d

    # Transform the point to the map frame
    try:
        pose_map = tf_buffer.transform(pose_3d, to_frame, rospy.Duration(1.0))
        
        # Extract x and y coordinates for 2D representation
        point_2d = Point()
        point_2d.x = pose_map.pose.position.x
        point_2d.y = pose_map.pose.position.y
        
        return point_2d
    
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(f"TF2 error: {e}")
        return None

# Example usage
if __name__ == '__main__':
    rospy.init_node('point_3d_to_2d_converter')
    
    # Example 3D point in camera frame
    point_3d = Point(1.0, 2.0, 3.0)
    
    point_2d = point_3d_to_2d(point_3d, "camera_link", "map")
    
    if point_2d:
        rospy.loginfo(f"2D point in map frame: x={point_2d.x}, y={point_2d.y}")
