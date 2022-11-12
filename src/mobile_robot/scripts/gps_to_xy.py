import rospy
import math
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from typing import Tuple
from calc.gps_to_xy import *

xy_pub = rospy.Publisher('/gps/xy', Vector3, queue_size=1000)

def publish_gps_to_xy(gps_data: NavSatFix):
    global xy_pub
    try:
        robot_pos = calculate_relative_to_xy(gps_data)
        xy_pub.publish(robot_pos)
    except Exception as e:
        print(f'Cannot convert gps to xy caused by {str(e)}')

if __name__ == '__main__':
    rospy.init_node('gps_to_xy_converter')
    rospy.Subscriber('/mobile_robot_gps/fix', NavSatFix,
                    publish_gps_to_xy)

    rospy.spin()
