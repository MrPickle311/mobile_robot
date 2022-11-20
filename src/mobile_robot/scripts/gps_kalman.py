import rospy
import math
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from typing import Tuple
from calc.gps_to_xy import *

class GPSFilter:
    def __init__(self):
        self.is_filter_initialized = False
        self.xy_sub =  rospy.Subscriber('/gps/xy', Vector3Stamped,
                    self.predict_and_update)
        self.xy_filtered_pub = rospy.Publisher('/gps/xy_filtered', Vector3Stamped, queue_size=1000)
        
        self.current_x_estimation = 0
        self.x_estimation_prognose = 0
        self.x_estimation_prognose_uncertainlity = 0.003
        self.curr_x_uncertainlity = 0
        
        self.current_y_estimation = 0
        self.y_estimation_prognose = 0
        self.y_estimation_prognose_uncertainlity = 0.003
        self.curr_y_uncertainlity = 0
        
        self.process_noise = 0.001
        self.xy_measurement_uncertainlity = 0.005 # orignal signal fit
    
    def predict_and_update(self, gps_data: Vector3Stamped):
        if not self.is_filter_initialized:
            self.initialize_filter(gps_data)
        
        self.predict()
        self.update(gps_data)
        
        msg = Vector3Stamped()
        msg.header = gps_data.header
        msg.vector.x = self.current_x_estimation
        msg.vector.y = self.current_y_estimation
        self.xy_filtered_pub.publish(msg)
    
    def predict(self):
        self.x_estimation_prognose = self.current_x_estimation
        self.x_estimation_prognose_uncertainlity = self.curr_x_uncertainlity + self.process_noise
        
        self.y_estimation_prognose = self.current_y_estimation
        self.y_estimation_prognose_uncertainlity = self.curr_y_uncertainlity + self.process_noise
    
    def update(self, gps_data: Vector3Stamped):
        x_kalman_gain = self.x_estimation_prognose_uncertainlity / (self.x_estimation_prognose_uncertainlity + self.xy_measurement_uncertainlity)
        x_innovation = gps_data.vector.x - self.x_estimation_prognose
        self.current_x_estimation = self.x_estimation_prognose + x_kalman_gain * x_innovation
        self.curr_x_uncertainlity = (1 - x_kalman_gain) * self.x_estimation_prognose_uncertainlity

        y_kalman_gain = self.y_estimation_prognose_uncertainlity / (self.y_estimation_prognose_uncertainlity + self.xy_measurement_uncertainlity)
        y_innovation = gps_data.vector.y - self.y_estimation_prognose
        self.current_y_estimation = self.y_estimation_prognose + y_kalman_gain * y_innovation
        self.curr_y_uncertainlity = (1 - y_kalman_gain) * self.y_estimation_prognose_uncertainlity
    
    def initialize_filter(self, data: Vector3Stamped):
        self.current_x_estimation = data.vector.x
        self.current_y_estimation = data.vector.y
        self.is_filter_initialized = True
        rospy.loginfo('Filter initialized')


if __name__ == '__main__':
    rospy.init_node('gps_filter')
    gps_filter = GPSFilter()
    rospy.spin()
