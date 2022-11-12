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

window_size = int(sys.argv[1])
deq = collections.deque(maxlen=window_size)
pub = rospy.Publisher(f'/heading/filtered/maf_{window_size}', Float32, queue_size=1000)
b = np.ones(window_size) / window_size

def filter(heading_data: Float32):
    global pub, window_size, b, deq
    deq.append(heading_data.data)
    if len(deq) < window_size:
        return
    
    listed_deq = list(deq)[-window_size:] # last L elements
    last_measurement = 1/window_size*(sum(listed_deq))
    
    msg = Float32()
    msg.data = last_measurement
    pub.publish(msg)


rospy.init_node(f'maf_filter_{window_size}')
rospy.Subscriber('/imu/compass_heading', Float32,filter)

rospy.spin()