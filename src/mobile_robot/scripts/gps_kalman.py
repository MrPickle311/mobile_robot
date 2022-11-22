import rospy
import math
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from typing import Tuple
from calc.gps_to_xy import *
import numpy as np

class GPSFilter:
    def __init__(self):
        self.is_filter_initialized = False
        self.xy_sub =  rospy.Subscriber('/gps/xy', Vector3Stamped,
                    self.predict_and_update)
        self.xy_filtered_pub = rospy.Publisher('/gps/xy_filtered', Vector3Stamped, queue_size=1000)
        
        self.X = np.matrix([[0],
                            [0]])
        self.P = np.matrix([[0.0, 0],
                            [0, 0.0]])

        self.I = np.eye(2)
        self.H = np.matrix([[1, 0],
                            [0, 1]])
        self.R = np.matrix([[0.005, 0],
                            [0, 0.005]])
        self.u = np.matrix([[0]])
        self.Q = np.array([[0.0008, 0],
                           [0, 0.0008]])
        self.F = np.matrix([[1,0],
                            [0,1]])
        self.G = np.matrix([[0],
                            [0]])
            
    def predict_and_update(self, gps_data: Vector3Stamped):
        if not self.is_filter_initialized:
            self.initialize_filter(gps_data)
            return
        
        measurement = np.matrix([[gps_data.vector.x],
                                 [gps_data.vector.y]])
        
        
        self.predict()
        self.update(measurement)
        
        msg = Vector3Stamped()
        msg.header = gps_data.header
        msg.vector.x = self.X[0,0]
        msg.vector.y = self.X[1,0]
        print(self.X[1,0])
        self.xy_filtered_pub.publish(msg)
    
    def predict(self):
        self.X = self.F*self.X + self.G*self.u
        self.P = self.F * self.P * np.transpose(self.F) + self.Q
    
    def update(self, measurement):
        innovation = measurement - self.H*self.X
        kallman_gain = self.P*np.transpose(self.H)* np.linalg.inv(self.H*self.P*np.transpose(self.H)+ self.R)
        self.X = self.X + kallman_gain*innovation
        S = self.I - kallman_gain*self.H
        self.P = S*self.P*np.transpose(S) + kallman_gain*self.R * np.transpose(kallman_gain)
    
    def initialize_filter(self, data: Vector3Stamped):
        self.X = np.matrix([[data.vector.x],
                            [data.vector.y]])
        self.is_filter_initialized = True
        rospy.loginfo('Filter initialized')


if __name__ == '__main__':
    rospy.init_node('gps_filter')
    gps_filter = GPSFilter()
    rospy.spin()
