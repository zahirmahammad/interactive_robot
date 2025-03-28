import rospy
import json
from nav_msgs.msg import Odometry

def save_odom_data(msg, file_name):
    """Extracts and saves odometry data to a JSON file once."""
    data = {
        "position": {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z
        },
        "orientation": {
            "x": msg.pose.pose.orientation.x,
            "y": msg.pose.pose.orientation.y,
            "z": msg.pose.pose.orientation.z,
            "w": msg.pose.pose.orientation.w
        }
    }
    with open(file_name, 'w') as f:
        json.dump(msg.pose, f, indent=4)
    rospy.loginfo("Odometry data saved in JSON format.")
    rospy.signal_shutdown("Odometry data recorded")

def main():
    rospy.init_node('odom_saver')
    file_name = input("Enter the filename to save odometry data (e.g., odom_data.json): ")
    msg = rospy.wait_for_message('/odom', Odometry)
    save_odom_data(msg, file_name)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
