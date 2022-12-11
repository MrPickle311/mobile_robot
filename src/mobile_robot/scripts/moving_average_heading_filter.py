import rospy
import math
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
import collections
import numpy as np
import scipy.signal as sig
import scipy.linalg as linalg
import sys

# max_window_size = int(sys.argv[1])
max_window_size = 35
deq_x = collections.deque(maxlen=max_window_size)
deq_y = collections.deque(maxlen=max_window_size)
# pub = rospy.Publisher(f'/heading/filtered/maf_{max_window_size}', Float32, queue_size=1000)
pub = rospy.Publisher(f'/heading/filtered/maf', Float32, queue_size=1000)
b = np.ones(max_window_size) / max_window_size
current_angular_velocity = 0
MAX_ANGULAR_VELOCITY = 1.0

def update_current_angular_velocity(msg: Twist):
    global current_angular_velocity
    current_angular_velocity = msg.angular.z

def distance_from_1_0(x, y):
    # 1 + 0j
    return math.sqrt((x - 1)**2 + (y - 0)**2)

def filter(heading_data: Float32):
    global pub, max_window_size, b, deq_x,deq_y, current_angular_velocity, MAX_ANGULAR_VELOCITY
    heading = heading_data.data
    x = math.cos(heading)
    y = math.sin(heading)
    deq_x.append(x)
    deq_y.append(y)
    if len(deq_x) < max_window_size:
        return
    
    current_window_size = int(max_window_size * ((MAX_ANGULAR_VELOCITY - current_angular_velocity) / MAX_ANGULAR_VELOCITY) ** 2 )
    print(current_window_size)
    
    # y = 0.05
    # x = 0.99875
    if distance_from_1_0(x,y) <= 0.05:
        current_window_size = 1
    
    if current_window_size < 1:
        current_window_size = 1
    
    listed_deq_x = list(deq_x)[-current_window_size:]
    last_measurement_x = (sum(listed_deq_x)) / current_window_size
    
    listed_deq_y = list(deq_y)[-current_window_size:]
    last_measurement_y = (sum(listed_deq_y)) / current_window_size
    
    msg = Float32()
    msg.data = math.fmod(np.angle(complex(last_measurement_x,last_measurement_y)) + 2*math.pi,2*math.pi) 
    pub.publish(msg)


# rospy.init_node(f'maf_filter_{max_window_size}')
rospy.init_node(f'maf_filter')
rospy.Subscriber('compass_heading', Float32,filter)
rospy.Subscriber('/cmd_vel', Twist,update_current_angular_velocity)

rospy.spin()