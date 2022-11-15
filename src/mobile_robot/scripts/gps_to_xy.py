import rospy
import math
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from typing import Tuple
from calc.gps_to_xy import *

xy_pub = rospy.Publisher('/gps/xy', Vector3Stamped, queue_size=1000)

def publish_gps_to_xy(gps_data: NavSatFix):
    global xy_pub
    try:
        result = Vector3Stamped()
        result.vector = calculate_relative_to_xy(gps_data)
        result.header = gps_data.header 
        xy_pub.publish(result)
    except Exception as e:
        print(f'Cannot convert gps to xy caused by {str(e)}')

if __name__ == '__main__':
    rospy.init_node('gps_to_xy_converter')
    rospy.Subscriber('/mobile_robot_gps/fix', NavSatFix,
                    publish_gps_to_xy)

    rospy.spin()
