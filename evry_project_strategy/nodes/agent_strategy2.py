#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

from evry_project_plugins.srv import DistanceToFlag


class Robot:
    def __init__(self, robot_name):
        """Constructor of the class Robot"""
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0  # Sonar distance
        self.x, self.y = 0.0, 0.0  # Robot's position
        self.yaw = 0.0  # Robot's yaw angle
        self.robot_name = robot_name

        # ROS Publishers and Subscribers
        rospy.Subscriber(f"{self.robot_name}/sensor/sonar_front", Range, self.callbackSonar)
        rospy.Subscriber(f"{self.robot_name}/odom", Odometry, self.callbackPose)
        self.cmd_vel_pub = rospy.Publisher(f"{self.robot_name}/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        """Callback for sonar sensor data"""
        self.sonar = msg.range

    def get_sonar(self):
        """Get the current sonar distance"""
        return self.sonar

    def callbackPose(self, msg):
        """Callback for odometry data"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, yaw = euler_from_quaternion(quaternion_list)
        self.yaw = yaw

    def get_robot_pose(self):
        """Get the robot's position and orientation"""
        return self.x, self.y, self.yaw

    def constraint(self, val, min_val=-2.0, max_val=2.0):
        """Constrain a value within a specified range"""
        return max(min(val, max_val), min_val)

    def set_speed_angle(self, linear, angular):
        """Set linear and angular velocities"""
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear)
        cmd_vel.angular.z = self.constraint(angular, min_val=-1, max_val=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        """Get the distance to the robot's designated flag"""
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D(x=self.x, y=self.y)
            result = service(pose, int(self.robot_name[-1]))
            return result.distance
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


def get_flag_position(robot_id):
    """Retrieve the position of the designated flag for each robot"""
    if robot_id == 1:
        return -21.21320344, 21.21320344  # Flag 1 position
    elif robot_id == 2:
        return 21.21320344, 21.21320344  # Flag 2 position
    elif robot_id == 3:
        return 0, -30  # Flag 3 position
    else:
        rospy.logwarn(f"Unknown robot ID: {robot_id}")
        return 0, 0


def calculate_angle_to_flag(robot):
    """Calculate the angle the robot needs to turn to face its designated flag"""
    flag_x, flag_y = get_flag_position(int(robot.robot_name[-1]))  # Determine flag based on robot ID
    delta_x = flag_x - robot.x
    delta_y = flag_y - robot.y
    desired_angle = math.atan2(delta_y, delta_x)  # Use math.atan2
    return desired_angle - robot.yaw


def avoid_obstacles(robot):
    """Handle obstacle avoidance based on sonar data"""
    sonar_distance = robot.get_sonar()
    if sonar_distance < 2.0:
        rospy.loginfo(f"{robot.robot_name}: Obstacle detected at {sonar_distance}m. Adjusting course.")
        return 0.5, 3.0 if robot.yaw < 0 else -3.0  # Slow down and turn
    return None  # No obstacle detected


def move_to_flag(robot, kp):
    """Move robot towards its designated flag using PID-based speed control"""
    distance_to_flag = robot.getDistanceToFlag()
    rospy.loginfo(f"{robot.robot_name}: Distance to flag = {distance_to_flag}")
    if distance_to_flag < 2.0:
        rospy.loginfo(f"{robot.robot_name}: Flag reached!")
        return 0.0, 0.0  # Stop the robot
    velocity = kp * distance_to_flag
    velocity = robot.constraint(velocity, max_val=2.0)  # Constrain velocity

    # Reorient toward the flag after avoiding obstacles
    angle_to_flag = calculate_angle_to_flag(robot)
    angle = robot.constraint(angle_to_flag, min_val=-1.0, max_val=1.0)

    return velocity, angle


def run_demo():
    """Main loop for robot strategy"""
    robot_name = rospy.get_param("~robot_name")
    kp = rospy.get_param("~kp", 1.0)  # PID proportional gain
    robot = Robot(robot_name)
    rospy.loginfo(f"Robot {robot_name} is starting...")

    # Timing strategy to stagger robot starts
    rospy.sleep(3 * int(robot_name[-1]))

    while not rospy.is_shutdown():
        # Handle obstacle avoidance
        obstacle_response = avoid_obstacles(robot)
        if obstacle_response:
            velocity, angle = obstacle_response
        else:
            # If no obstacles, reorient and move toward the flag
            velocity, angle = move_to_flag(robot, kp)

        # Publish the velocity and angle
        robot.set_speed_angle(velocity, angle)
        rospy.sleep(0.5)


if __name__ == "__main__":
    rospy.init_node("Controller", anonymous=True)
    try:
        run_demo()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down the robot controller.")
