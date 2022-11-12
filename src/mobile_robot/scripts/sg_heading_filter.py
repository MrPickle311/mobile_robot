import rospy
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
import collections
import scipy.signal as sig
import sys

window_size = int(sys.argv[1])
deq = collections.deque(maxlen=window_size)
pub = rospy.Publisher(f'/heading/filtered/sg_{window_size}', Float32, queue_size=1000)
b = sig.savgol_coeffs(window_size, 3)

def filter(heading_data: Float32):
    global pub, window_size, b, deq
    deq.append(heading_data.data)
    if len(deq) < window_size:
        return
    
    listed_deq = list(deq)
    filtered_data = sig.savgol_filter(listed_deq,window_size,3)
    
    msg = Float32()
    msg.data = filtered_data[-1]
    pub.publish(msg)


rospy.init_node(f'maf_filter_{window_size}')
rospy.Subscriber('/imu/compass_heading', Float32,filter)

rospy.spin()