import rospy
import math
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from typing import Tuple

EARTH_RADIUS = 6378.137  # kilometers

def calculate_xy(latt: float, long: float,fi_0: float) -> Tuple[float,float]:
    global EARTH_RADIUS
    x = EARTH_RADIUS*math.radians(latt)*1000
    y = EARTH_RADIUS*math.radians(long)*math.cos(math.radians(fi_0))*1000
    
    # rotate axes
    x *= -1 # it must be almost equal with odom, axes direction must agree with odometry
    return x, y
    
def calculate_relative_to_xy(desired_point: NavSatFix) -> Vector3:
    base_latt = float(rospy.get_param('/mobile_robot_diff_drive_controller/base_latt'))
    base_long = float(rospy.get_param('/mobile_robot_diff_drive_controller/base_long'))

    fi_0 = base_latt  # where φ0 denotes a latitude close to the center of your map.

    new_pos = Vector3()
    new_pos.x , new_pos.y = calculate_xy(desired_point.latitude, desired_point.longitude, fi_0)

    origin = Vector3()
    origin.x, origin.y = calculate_xy(base_latt,base_long,fi_0)

    new_pos.x = origin.x - new_pos.x
    new_pos.y = origin.y - new_pos.y
    
    return new_pos