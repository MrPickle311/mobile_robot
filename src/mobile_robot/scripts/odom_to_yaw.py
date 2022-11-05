from tf.transformations import euler_from_quaternion
import rospy
from std_msgs.msg import *
from nav_msgs.msg import *

def transform_odom_quat_to_yaw(msg: Odometry):
    yaw_odom_pub = rospy.Publisher('/mobile_robot/odom/yaw', Float32, queue_size=1000)
    quat = msg.pose.pose.orientation
    _,_,res = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
    pub_msg = Float32()
    pub_msg.data = res
    yaw_odom_pub.publish(pub_msg)

rospy.init_node('odom_to_yaw')
rospy.Subscriber('mobile_robot/odom', Odometry, transform_odom_quat_to_yaw)
rospy.spin()