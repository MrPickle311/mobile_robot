import rospy
import math
from sensor_msgs.msg import *
from nav_msgs.msg import *

EARTH_RADIUS = 6378.137  # kilometers


def get_drone_distance_from_miranda(gps_data: NavSatFix) -> float:
    global EARTH_RADIUS
    d_latt = math.radians(0 - gps_data.latitude)
    d_long = math.radians(0 - gps_data.longitude)
    a = math.sin(d_latt / 2) ** 2 + \
        math.cos(math.radians(0)) * math.cos(math.radians(gps_data.latitude)) * \
        math.sin(d_long / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = EARTH_RADIUS * c
    φ0 = 0  # where φ0 denotes a latitude close to the center of your map.
    print(f'y: {EARTH_RADIUS*math.radians(gps_data.latitude)*1000}')
    print(f'x: {EARTH_RADIUS*math.radians(gps_data.longitude)*math.cos(φ0)*1000}')
    print(f'Distance: {d*1000}')


def prnt_odom_cords(odom_pos: Odometry):
    print(
        f'Odometry x: {odom_pos.pose.pose.position.x} y: {odom_pos.pose.pose.position.y}')


rospy.init_node('xd')
rospy.Subscriber('/gps/fix', NavSatFix,
                 get_drone_distance_from_miranda)
# rospy.Subscriber('/odom', Odometry,
#                  prnt_odom_cords)

rospy.spin()
