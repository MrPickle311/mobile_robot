from std_msgs.msg import *
import rospy, actionlib
from mobile_robot.msg import *

class MissionReader:
    def __init__(self):
        self.sub = rospy.Subscriber('/mobile_robot/mission', String,
                        self.service_mission_execution)
        self.mission_client = actionlib.SimpleActionClient(
        '/mission_plan', MissionPlanAction)
    
    def service_mission_execution(self, msg: String):
        path = msg.data
        plan = MissionPlanGoal()
        
        with open(path) as file:
            lines = file.readlines()
        
        for line in lines:
            point = PointTravel()
            records = line.split(',')
            point.desired_point.latitude = float(records[0])
            point.desired_point.longitude = float(records[1])
            point.K_1 = float(records[2])
            point.K_2 = float(records[3])
            point.K_3 = float(records[4])
            point.gentleness = int(records[5])
            point.move_accuracy = float(records[6])
            point.rotation_accuracy = float(records[7])
            plan.points.append(point)
        
        for point in plan.points:
            print(f'point.desired_point.latitude: {point.desired_point.latitude}')
            print(f'point.desired_point.longitude: {point.desired_point.longitude}')
            print(f'point.K_1: {point.K_1}')
            print(f'Point.K_2: {point.K_2}')
            print(f'Point.K_3: {point.K_3}')
            print(f'gentleness {point.gentleness}')
            print(f'move_accuracy {point.move_accuracy}')
            print(f'rotation_accuracy {point.rotation_accuracy}')
        
        self.mission_client.send_goal(plan)
        
    

if __name__ == '__main__':
    rospy.init_node('mission_reader')
    reader = MissionReader()
    rospy.spin()
    