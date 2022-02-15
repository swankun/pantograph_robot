import rospy
from geometry_msgs.msg import Twist

from kinematics import PantographKinematics

class StrikerTeleopROS(object):

    def __init__(self):
        self.motor_publishers = [
            rospy.Publisher("/theta1_controller/command", MotorCommands, queue_size=1,
            rospy.Publisher("/theta4_controller/command", MotorCommands, queue_size=1
        ]
        self.twist_subscriber = rospy.Subscriber("striker_twist", Twist, self.twist_callback)

    def twist_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        