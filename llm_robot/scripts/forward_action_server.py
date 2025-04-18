import rospy
import actionlib
from geometry_msgs.msg import Twist
from llm_robot.msg import MoveForwardAction, MoveForwardFeedback, MoveForwardResult

class MoveForwardServer:
    def __init__(self):
        print("Hi")
        self.server = actionlib.SimpleActionServer('move_forward', MoveForwardAction, self.execute_cb, auto_start=False)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.feedback = MoveForwardFeedback()
        self.result = MoveForwardResult()

        self.server.start()
        print("Server Started")


    def execute_cb(self, goal):
        rate = rospy.Rate(10)
        vel = Twist()
        vel.linear.x = 0.2

        distance_moved = 0.0
        t0 = rospy.Time.now().to_sec()
        print("in callback")

        while distance_moved < goal.distance:
            if self.server.is_preempt_requested():
                self.cmd_pub.publish(Twist())
                self.server.set_preempted()
                print("Goal Prempted")
                return
            
            self.cmd_pub.publish(vel)
            t1 = rospy.Time.now().to_sec()
            distance_moved = vel.linear.x * (t1-t0)

            self.feedback.current_distance = distance_moved
            self.server.publish_feedback(self.feedback)
            rate.sleep()

        self.cmd_pub.publish(Twist())
        self.result.success = True
        self.server.set_succeeded(self.result)
        print("Goal succeeded")

            
if __name__=="__main__":
    rospy.init_node("move_forward_server")
    server = MoveForwardServer()
    rospy.spin()

# #!/usr/bin/env python3

# import rospy
# import actionlib
# from geometry_msgs.msg import Twist
# from your_package.msg import MoveForwardAction, MoveForwardFeedback, MoveForwardResult

# class MoveForwardServer:
#     def __init__(self):
#         self.server = actionlib.SimpleActionServer('move_forward', MoveForwardAction, self.execute_cb, False)
#         self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#         self.feedback = MoveForwardFeedback()
#         self.result = MoveForwardResult()
#         self.server.start()
#         rospy.loginfo("Move Forward Action Server Started")

#     def execute_cb(self, goal):
#         rate = rospy.Rate(10)
#         vel = Twist()
#         vel.linear.x = 0.2  # forward speed

#         distance_moved = 0.0
#         t0 = rospy.Time.now().to_sec()

#         while distance_moved < goal.distance:
#             if self.server.is_preempt_requested():
#                 self.cmd_pub.publish(Twist())  # stop robot
#                 self.server.set_preempted()
#                 rospy.loginfo("Goal preempted")
#                 return

#             self.cmd_pub.publish(vel)
#             t1 = rospy.Time.now().to_sec()
#             distance_moved = 0.2 * (t1 - t0)

#             self.feedback.current_distance = distance_moved
#             self.server.publish_feedback(self.feedback)
#             rate.sleep()

#         self.cmd_pub.publish(Twist())  # stop
#         self.result.success = True
#         self.server.set_succeeded(self.result)
#         rospy.loginfo("Goal succeeded")

# if __name__ == '__main__':
#     rospy.init_node('move_forward_server')
#     server = MoveForwardServer()
#     rospy.spin()
