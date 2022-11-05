from abc import ABC, abstractmethod
import rospy
import math , time
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
import actionlib
from mobile_robot.msg import MissionPlanAction, MissionPlanGoal, MissionPlanResult, MissionPlanFeedback
from mobile_robot.msg import RotationAction, RotationGoal, RotationResult, RotationFeedback
from mobile_robot.msg import MovementAction, MovementGoal, MovementFeedback, MovementResult
from typing import Union
import matplotlib.pyplot as plt
import numpy

class PreemptException(Exception):
    ...

times = []
left_ang = []

EARTH_RADIUS = 6378.137  # kilometers


class MobileRobotPositionKeeper:
    def __init__(self):
        self.heading = 0
        self.position = Vector3()
        rospy.Subscriber('/gps/xy', Vector3, self.update_robot_xy)
        rospy.Subscriber('imu/compass_heading', Float32, self.update_heading)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)

    @staticmethod
    def get_distance_between_points(gps_point_1: NavSatFix, gps_point_2: NavSatFix) -> float:
        global EARTH_RADIUS

        d_latt = math.radians(gps_point_1.latitude - gps_point_2.latitude)
        d_long = math.radians(gps_point_1.longitude - gps_point_2.longitude)
        a = math.sin(d_latt / 2) ** 2 + \
            math.cos(math.radians(gps_point_1.latitude)) * math.cos(math.radians(gps_point_2.latitude)) * \
            math.sin(d_long / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = EARTH_RADIUS * c
        return d*1000

    @staticmethod
    def vector_len(vector: Vector3) -> float:
        return math.sqrt(math.pow(vector.x, 2) + math.pow(vector.y, 2))

    def update_robot_xy(self, position: Vector3):
        self.position = position

    def update_heading(self, heading: Float32):
        self.heading = heading.data

    def get_displacement_vector(self, destination_point: Point) -> Vector3:
        v_disp = Vector3()
        v_disp.x = destination_point.x - self.position.x
        v_disp.y = destination_point.y - self.position.y
        return v_disp

    def get_rotation_of_vector(self, vector: Vector3):
        result = math.atan2(vector.y, vector.x)
        return math.fmod(result + 2 * math.pi, 2 * math.pi)

    def get_rot_diff_from_heading(self, rot: float) -> float:
        return rot - self.heading

    def get_absolute_rotation_to_point(self, point: Point) -> float:
        v_disp = self.get_displacement_vector(point)
        return self.get_rotation_of_vector(v_disp)

    def get_missing_rotation_to_point(self, point: Point) -> float:
        v_disp_rot = self.get_absolute_rotation_to_point(point)
        return self.get_rot_diff_from_heading(v_disp_rot)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

class VelocityExecutor(MobileRobotPositionKeeper, ABC):
    def move(self, movement_data: Union[MovementGoal,RotationGoal]):
        self.speed_up(movement_data)
        self.wait_for_reduction_step(movement_data)
        self.reduce(movement_data)
    
    @abstractmethod
    def speed_up(self, movement_data: Union[MovementGoal,RotationGoal]) -> None:
        ...

    @abstractmethod
    def wait_for_reduction_step(self, movement_data: Union[MovementGoal,RotationGoal]) -> None:
        ...
        
    @abstractmethod
    def reduce(self, movement_data: Union[MovementGoal,RotationGoal]) -> None:
        ...
    

class MovementExecutor(VelocityExecutor):
    _feedback = MovementFeedback()
    _result = MovementResult()
    CHANGES_COUNT = 10  # gentleness
    K1 = 0.1
    K2 = 0.2
    K3 = 0.8

    def __init__(self, name):
        super().__init__()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, MovementAction, execute_cb=self.move, auto_start=False)
        self._as.start()

    def get_left_distance(self, dest_point: Point) -> float:
        return self.vector_len(self.get_displacement_vector(dest_point))

    def get_traveled_distance(self, movement_data: MovementGoal) -> float:
        return movement_data.distance - self.get_left_distance(movement_data.desired_point)

    def get_reduce_distance(self, movement_data: MovementGoal) -> float:
        return movement_data.distance * self.K3

    def get_post_reduce_distance(self, movement_data: MovementGoal) -> float:
        return movement_data.distance - self.get_reduce_distance(movement_data)

    def get_v_max(self, movement_data: MovementGoal) -> float:
        return self.K1 * movement_data.distance

    def get_stabilize_distance(self, movement_data: MovementGoal) -> float:
        return self.K2 * movement_data.distance

    def speed_up(self, movement_data: MovementGoal):
        v_max = self.get_v_max(movement_data)
        stabilize_distance = self.get_stabilize_distance(movement_data)
        rospy.loginfo(
            f'Speeding up linear vel with: v_max: {v_max} stabilize_distance {stabilize_distance}')

        for change_step in range(self.CHANGES_COUNT):
            progress = (change_step+1)/self.CHANGES_COUNT
            vel_msg = Twist()
            vel_msg.angular = Vector3()
            vel_msg.linear.x = progress * v_max
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            rospy.loginfo(
                f'Changing linear velocity change_step {change_step}, linear.x: {vel_msg.linear.x} ')
            self.cmd_vel_pub.publish(vel_msg)

            while self.get_traveled_distance(movement_data) < (progress*stabilize_distance):
                ...

    def wait_for_reduction_step(self, movement_data: MovementGoal):
        post_reduce_distance = self.get_post_reduce_distance(movement_data)

        rospy.loginfo(
            f'Waiting for reduction step: with x: {movement_data.desired_point.x}, y: {movement_data.desired_point.y}, post_reduce_distance: {post_reduce_distance}')

        while not math.isclose(self.get_left_distance(movement_data.desired_point), post_reduce_distance, abs_tol=movement_data.accuracy):
            rospy.loginfo(
                f'Current distance left: {self.get_left_distance(movement_data.desired_point)} robot(x,y): ({self.position.x},{self.position.y})')
            rospy.sleep(0.1)

    def reduce(self, movement_data: MovementGoal):
        v_max = self.get_v_max(movement_data)
        reduce_distance = self.get_reduce_distance(movement_data)
        d_dist = self.get_post_reduce_distance(
            movement_data) / self.CHANGES_COUNT
        rospy.loginfo(
            f'Reducing linear vel with: reduce_distance {reduce_distance} d_dist {d_dist}, v_max: {v_max}, last_stop: {reduce_distance+d_dist*self.CHANGES_COUNT}')

        for change_step in range(self.CHANGES_COUNT + 1):
            def get_next_endstop():
                return reduce_distance + (d_dist*change_step)

            while not math.isclose(self.get_traveled_distance(movement_data), get_next_endstop(), abs_tol=movement_data.accuracy):
                ...

            progress = change_step/self.CHANGES_COUNT
            vel_msg = Twist()
            vel_msg.angular = Vector3()
            vel_msg.linear.x = (1 - progress) * v_max
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            rospy.loginfo(
                f'Changing linear velocity change_step {change_step}, linear.x: {vel_msg.linear.x}')
            self.cmd_vel_pub.publish(vel_msg)

        self.stop_robot()


class RotationExecutor(VelocityExecutor):
    _feedback = RotationFeedback()
    _result = RotationResult()

    CHANGES_COUNT = 10  # gentleness
    K1 = 0.1
    K2 = 0.2
    K3 = 0.8

    def __init__(self, name):
        super().__init__()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, RotationAction, execute_cb=self.move, auto_start=False)
        self._as.start()
    
    def get_omega_max(self,rotation_data: RotationGoal):
        return self.K1 * rotation_data.angle
    
    def get_stabilize_angle(self, rotation_data: RotationGoal):
        return self.K2 * rotation_data.angle
    
    def get_left_angle(self, dest_point: Point) -> float:
        return math.fabs(self.get_missing_rotation_to_point(dest_point))
    
    def get_reduce_angle(self, rotation_data: RotationGoal) -> float:
        return rotation_data.angle * self.K3

    def get_traveled_angle(self, rotation_data: RotationGoal) -> float:
        res = rotation_data.angle - self.get_left_angle(rotation_data.desired_point)
        # print(f'Traveled angle: {res}')
        return res

    def get_post_reduce_angle(self, rotation_data: RotationGoal) -> float:
        return rotation_data.angle - self.get_reduce_angle(rotation_data)

    def speed_up(self, rotation_data: RotationGoal):
        omega_max = self.get_omega_max(rotation_data)
        stabilize_angle = self.get_stabilize_angle(rotation_data)
        rospy.loginfo(
            f'Speeding up angular vel with: omega_max: {omega_max} stabilize_angle {stabilize_angle}')

        for change_step in range(self.CHANGES_COUNT):
            progress = (change_step+1)/self.CHANGES_COUNT
            vel_msg = Twist()
            vel_msg.linear = Vector3()
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = progress * omega_max
            vel_msg.angular.z *= rotation_data.direction
            rospy.loginfo(
                f'Changing angular velocity change_step {change_step}, angular.z: {vel_msg.angular.z} ')
            self.cmd_vel_pub.publish(vel_msg)

            while self.get_traveled_angle(rotation_data) < (progress*stabilize_angle):
                ...

    def wait_for_reduction_step(self, rotation_data: RotationGoal):
        post_reduce_angle = self.get_post_reduce_angle(rotation_data)
        rospy.loginfo(
            f'Waiting for reduction step with rotation_data.angle {rotation_data.angle}, heading: {self.heading}, post_reduce_angle: {post_reduce_angle}')

        while not math.isclose(self.get_left_angle(rotation_data.desired_point), post_reduce_angle , abs_tol=rotation_data.accuracy):
            rospy.loginfo(
                f'Current angle left: {self.get_left_angle(rotation_data.desired_point)} robot_heading: {self.heading}')
            rospy.sleep(0.1)

    def reduce(self, rotation_data: RotationGoal):
        reduce_time = time.time()
        global left_ang
        global times 
        omega_max = self.get_omega_max(rotation_data)
        reduce_angle = self.get_reduce_angle(rotation_data)
        d_angle = self.get_post_reduce_angle(rotation_data) / self.CHANGES_COUNT
        rospy.loginfo(
            f'Reducing angular vel with: reduce_angle: {reduce_angle}, d_angle: {d_angle}')

        for change_step in range(self.CHANGES_COUNT + 1):
            def get_next_endstop():
                res = reduce_angle + (d_angle*change_step)
                # print(f'Endstop angle: {res}')
                return res
            
            while not math.isclose(self.get_traveled_angle(rotation_data), get_next_endstop(), abs_tol=rotation_data.accuracy):
                left_ang.append(rotation_data.angle - self.get_traveled_angle(rotation_data))
                times.append(time.time() - reduce_time)
                # print(f'Diff {math.degrees(math.fabs(self.get_traveled_angle(rotation_data) - get_next_endstop()))} degrees')
                ...

            progress = change_step/self.CHANGES_COUNT
            vel_msg = Twist()
            vel_msg.linear = Vector3()
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = (1 - progress) * omega_max
            vel_msg.angular.z *= rotation_data.direction
            rospy.loginfo(
                f'Changing angular velocity change_step {change_step}, angular.z: {vel_msg.angular.z} ')
            self.cmd_vel_pub.publish(vel_msg)
            # PO OSTATNIM ENDSTOPIE CZEKAMY NA WYNIK OKRESLONY Z ZAŁOŻONĄ DOKŁADNOŚCIĄ
        self.stop_robot()


class MissionPlanner(MobileRobotPositionKeeper):
    _feedback = MissionPlanFeedback()
    _result = MissionPlanResult()
    ROT_EPS = math.radians(1) # last dopuszczalny 0,054680184
    MOV_EPS = 0.08 # meters 

    def __init__(self, name):
        super().__init__()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, MissionPlanAction, execute_cb=self.execute_mission, auto_start=False)
        self.rotation_client = actionlib.SimpleActionClient(
            '/rotation_process', RotationAction)
        self.movement_client = actionlib.SimpleActionClient(
            '/movement_process', MovementAction)
        self._as.start()

    def execute_mission(self, mission: MissionPlanGoal):
        success = True

        for point in mission.points:
            try:
                self.rotate_robot(point)
                # self.move_robot(point)
                # niech punkty mają swoje id
                self.publish_point_achieved(point)
            except PreemptException as e:
                rospy.loginfo('Preempted mission planner')
                self._as.set_preempted()
                success = False
                break

        if success:
            self.finish()

    def calculate_rotation_path(self, point: Point):
        v_disp_rot = self.get_absolute_rotation_to_point(point)

        # fi_12
        print(f'Init heading: {self.heading}')
        diff_rot = self.get_rot_diff_from_heading(v_disp_rot)

        direction = RotationGoal.LEFT

        if -2*math.pi < diff_rot < -1*math.pi:
            print('Case 1')
            diff_rot += 2*math.pi
            direction = RotationGoal.LEFT
        elif -1*math.pi < diff_rot < 0:
            print('Case 2')
            direction = RotationGoal.RIGHT
        elif 0 < diff_rot < math.pi:
            print('Case 3')
            direction = RotationGoal.LEFT
        elif math.pi < diff_rot < 2 * math.pi:
            print('Case 4')
            diff_rot = 2*math.pi - diff_rot
            direction = RotationGoal.RIGHT
        
        rospy.loginfo(
            f'Selected robot rotation v_disp_rot: {v_disp_rot}, diff_rot: {diff_rot}, direction: {direction}')

        return diff_rot, direction
    
    def rotate_robot(self, point: Point):
        diff_rot, direction = self.calculate_rotation_path(point)

        goal = RotationGoal()
        goal.accuracy = self.ROT_EPS
        goal.angle = math.fabs(diff_rot)
        goal.direction = direction
        goal.desired_point = point

        self.rotation_client.send_goal(goal)
        if self._as.is_preempt_requested():
            ...
        self.rotation_client.wait_for_result()

    def move_robot(self, point: Point):
        v_disp = self.get_displacement_vector(point)

        goal = MovementGoal()
        goal.accuracy = self.MOV_EPS
        goal.distance = self.vector_len(v_disp)
        goal.desired_point = point

        rospy.loginfo(
            f'Moving robot with v_disp(x,y): ({v_disp.x},{v_disp.y}), distance: {goal.distance}, goal(x,y): ({goal.desired_point.x},{goal.desired_point.y})')

        self.movement_client.send_goal(goal)
        if self._as.is_preempt_requested():
            ...
        self.movement_client.wait_for_result()

    def publish_point_achieved(self, point: Point):
        self._as.publish_feedback(self._feedback)
        ...

    def finish(self):
        rospy.loginfo('Mission finished')
        # self._as.set_succeeded(self._result)

def plot():
    global times
    global left_ang
    plt.plot(times,left_ang)
    sigma = numpy.std(left_ang)
    plt.grid(True)
    plt.title('Zależność brakującego kąta od czasu')
    plt.text(2.5,0.3,r'$\sigma=$' + f'{round(sigma,6)}')
    plt.ylabel('Brakujący kąt do pożądanej orientacji robota [rad]')
    plt.xlabel('Czas [s]')
    plt.show()

if __name__ == '__main__':
    rospy.init_node('mission_planner')
    planner_server = MissionPlanner('planner')
    rotation_server = RotationExecutor('/rotation_process')
    movement_server = MovementExecutor('/movement_process')
    rospy.sleep(1)
    rospy.loginfo('Started')

    mission = MissionPlanGoal()

    point1 = Point()
    point1.y = 0
    point1.x = -5

    # point2 = Point()
    # point2.x = 10
    # point2.y = 5

    # point3 = Point()

    # mission.points = [point1, point2, point3]
    mission.points = [point1]
    # print(planner_server.calculate_rotation_path(point1))
    
    planner_server.execute_mission(mission)
    # plot()

    rospy.spin()
