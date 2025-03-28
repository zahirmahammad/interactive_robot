import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Publisher for the gripper
pub_gripper = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)

home_pose = [0.0, -1.57, 1.57, 0.0, -0.01]  # joint pose and gripper pose
# gripper qpose [-0.02, 0.01]


def send_joint_trajectory(joint_angles, duration=2.0):
    """Publishes joint trajectory commands to the manipulator"""
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(3)  
    traj = JointTrajectory()
    traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']  # Modify based on your manipulator
    
    point = JointTrajectoryPoint()
    point.positions = joint_angles[:-1]
    point.time_from_start = rospy.Duration(duration)
    
    traj.points.append(point)
    pub.publish(traj)

    gripper_qpos = joint_angles[-1]

    """Controls the gripper to open or close."""
    # rospy.loginfo("Gripper " + ("Opening" if gripper_qpos>0 else "Closing"))
    traj = JointTrajectory()
    traj.joint_names = ['gripper']  # Modify based on your gripper setup
    
    point = JointTrajectoryPoint()
    point.positions = [gripper_qpos]  # Adjust values as needed
    point.time_from_start = rospy.Duration(1.0)
    
    traj.points.append(point)
    pub_gripper.publish(traj)
    rospy.sleep(1)

def pick_up():
    """Moves the manipulator to pick up position."""
    rospy.loginfo("Moving to pick-up position")
    pick_position = [0.0, 0.74, -0.74, 0.0, 0.01]  
    send_joint_trajectory(pick_position)
    rospy.sleep(1)
    send_joint_trajectory([0.0, 0.74, -0.74, 0.0, -0.01])
    rospy.sleep(1)
    send_joint_trajectory(home_pose)


def deliver():
    """Moves the manipulator to deliver position."""
    rospy.loginfo("Moving to delivery position")
    deliver_position = [0.0, 0.74, -0.74, 0.0, -0.01]  
    send_joint_trajectory(deliver_position)
    rospy.sleep(1)
    send_joint_trajectory([0.0, 0.74, -0.74, 0.0, 0.01])
    rospy.sleep(1)
    send_joint_trajectory(home_pose)

def main():
    rospy.init_node('manipulator_control')
    rospy.loginfo("Manipulator control node started")
    
    rospy.loginfo("Manipulator home")
    send_joint_trajectory(home_pose)
    rospy.sleep(1)


    # pick_up()
    # rospy.sleep(1)
    # deliver()
    # rospy.loginfo("Task completed")
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass