import rospy
import math
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *

EARTH_RADIUS = 6378.137  # kilometers

xy_pub = rospy.Publisher('/gps/xy', Vector3, queue_size=1000)


def convert_gps_to_xy(gps_data: NavSatFix):
    global EARTH_RADIUS, xy_pub
    φ0 = 0  # where φ0 denotes a latitude close to the center of your map.
    vec = Vector3()
    vec.x = EARTH_RADIUS*math.radians(gps_data.latitude)*1000
    vec.y = EARTH_RADIUS*math.radians(gps_data.longitude)*math.cos(φ0)*1000
    vec.z = 0
    vec.y *= -1  # it must be almost equal with odom, axes direction must agree with odometry
    xy_pub.publish(vec)


rospy.init_node('gps_to_xy_converter')
rospy.Subscriber('/mobile_robot_gps/fix', NavSatFix,
                 convert_gps_to_xy)

rospy.spin()
