#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.

# Author: Xenia Dela Cueva

# Creates an exploring robot. Deployed into an unknown environment, it will map out the
# environment as it moves. Will initially be in random walk mode, but will follow the wall to its right
# as it explores (if it senses one)

# Import of python modules.
import math  # use of pi.
import random
import numpy as np
import time
# import of relevant libraries.
import rospy  # module for ROS APIs
# message type for cmd_vel
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3
from sensor_msgs.msg import LaserScan  # message type for scan
from std_srvs.srv import SetBool, SetBoolResponse
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
import tf


# Constants.
# Topic names
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_ODOM_TOPIC = '/odom'
DAFAULT_BASE_LINK_TOPIC = '/base_link'
DEFAULT_SCAN_TOPIC = '/base_scan'
DEFAULT_MAP_TOPIC = '/map'

# Frequency at which the loop operates
FREQUENCY = 10  # Hz.

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = 0.2  # m/s
ANGULAR_VELOCITY = math.pi / 4  # rad/s


THETA = math.pi/4
DEFAULT = 0
RADIUS = 2

# Field of view in radians that is checked in front of the robot (feel free to tune)
MIN_SCAN_ANGLE_RAD = -135.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD = -45.0 / 180 * math.pi

MIN_SCAN_ANGLE_RAD_FRONT = -10.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD_FRONT = +10.0 / 180 * math.pi

DIS_THRESHHOLD = 0.2
# if the robot is at or closer to this threshold to the goal point, then we would have it rotate

# MAP specs: map_res, map_width, map_height, map_og_x, map_og_y
MAP_RESOLUTION = 0.1
MAP_WIDTH = 20
MAP_HEIGHT = 20
MAP_X = 0
MAP_Y = 0
OBSTACLE_VAL = 100
FREESPACE_VAL = 0

# What I assume to be the traslation of map origin from the odom origin (in reference to odom)
# feel free to tune, the robot doesn't know its actual place in world, you kind of do?
x_translation = -5
y_translation = -5
z_translation = 0


# proportions for PD controller
KP = 1
KD = 20

# Threshold of minimum clearance distance (feel free to tune)
# m, threshold distance, should be smaller than range_max
MIN_THRESHOLD_DISTANCE = 0.5
MIN_THRESHOLD_DISTANCE_FRONT = 0.5


# Finite State Machine
class fsm(Enum):
    WAITING_FOR_LASER = 0
    STOP = 1
    FOLLOW_WALL = 3
    ROTATE = 2
    RANDOMIZE = 4
    MOVE_FORWARD = 5

# Proportional Derivative (PD) controller. Refer to README as to why Integral wasn't included


class PD:
    def __init__(self, kp, kd):
        """Constructor."""
        self._p = kp
        self._d = kd
        self._err_prev = None

    def step(self, err, dt):
        """to calculate actuation command (angular velocity for robot)."""

        u = 0
        # self._err_sum += err
        if self._err_prev:
            # PD controller, no Integral component
            # u = self._p * err + self._d * (err - self._err_prev) / dt + self._i * dt * self._err_sum + self._k
            u = self._p * err + self._d * (err - self._err_prev) / dt

        self._err_prev = err
        return u

    def reset(self):
        """to reset"""
        self._err_prev = None
        self._err_sum = 0


class Grid:  # reads the occupancy grid we already upoloaded
    def __init__(self, width, height, resolution, map_og_x, map_og_y):
        self.rows = int(height/resolution)  # rasterized
        self.cols = int(width/resolution)
        self.resolution = resolution
        self.width = width  # meters
        self.height = height  # meters
        self.pos_x = map_og_x
        self.pos_y = map_og_y
        self.grid_array = []

    def initialize_map(self):
        """ Making a grid as unknown"""
        grid = []
        # making a 2D array with it
        for i in range(0, self.rows):
            each_row = [-1] * self.cols  # making -1 as unknown
            grid.append(each_row)

        grid = np.array(grid)
        self.grid_array = grid

    def cell_at(self, x, y):  # returns value of cell
        """ To return the value at that cell"""
        if (0 <= x < self.cols) and (0 <= y < self.rows):
            return self.grid_array[y][x]  # row, col

    def set_cell(self, x, y, new_val):
        """ To set the cell to new val"""
        if (0 <= y < self.rows) and (0 <= x < self.cols):
            self.grid_array[y][x] = new_val

    # since indices start at 0 index for rows and cols end in their length -1
    def cell_exists(self, x, y):
        """ to check if a cell exists"""
        if (0 <= y < self.rows) and (0 <= x < self.cols):
            return True
        return False


class Explorer():

    # default (0,0) position (in reference to odom) and (0,0,0) orientation of robot in reference to environment
    # again we assume the robot doesn't know its environment
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, x=DEFAULT,
                 y=DEFAULT, orient_x=DEFAULT, orient_y=DEFAULT, orient_z=DEFAULT, map_res=MAP_RESOLUTION,
                 map_width=MAP_WIDTH, map_height=MAP_HEIGHT, map_x=MAP_X, map_y=MAP_Y, kp=KP, kd=KD,
                 desired_threshold_distance=MIN_THRESHOLD_DISTANCE, desired_threshold_distance_front=MIN_THRESHOLD_DISTANCE_FRONT,
                 scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD], scan_angle_front=[MIN_SCAN_ANGLE_RAD_FRONT, MAX_SCAN_ANGLE_RAD_FRONT]):

        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher(
            DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        self.map_publisher = rospy.Publisher(
            DEFAULT_MAP_TOPIC, OccupancyGrid, queue_size=1)
        self._map_meta_data_pub = rospy.Publisher(
            'map_metadata', MapMetaData, queue_size=1, latch=True)

        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = rospy.Subscriber(
            DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            DEFAULT_ODOM_TOPIC, Odometry, self.odom_callback)  # odom
        self.listener = tf.TransformListener()

        self._on_off_service = rospy.Service(
            'on_off', SetBool, self._turn_on_off_callback)  # finite state machine

        # Parameters.
        self.linear_velocity = linear_velocity  # Constant linear velocity set.
        self.angular_velocity = angular_velocity

        # orientations
        self.or_x = orient_x
        self.or_y = orient_y
        self.or_z = orient_z

        # positions
        self.p_x = x
        self.p_y = y

        # Robot's Map
        # (self, width, height, resolution, map_og_x, map_og_y)
        self.grid = Grid(map_width, map_height, map_res, map_x, map_y)

        # fsm variable.
        self._fsm = fsm.STOP

        # its controller
        self.pid = PD(kp, kd)  # kp and kd parameter
        self.u = 0  # used for angular velocity

        # in accounting for left turns
        # range of angles for distance scans for right side (min_angle, max_angle)
        self.scan_angle = scan_angle
        # range of angles for distance scans for front (min_angle, max_angle)
        self.scan_angle_front = scan_angle_front
        # self.front_distance_angle keeps track of angle of min distance in front
        self.front_distance_angle = 0

        self.front_distance = 0  # self.front_distance keeps track of the min distance in front
        self.desired_threshold_distance = desired_threshold_distance
        # we start that we assume a wall is very far from the front, and this will get updated in laser scan
        self.desired_threshold_distance_front = desired_threshold_distance_front * 2

        self._close_obstacle = False

        pass

    # Moving functions
    def _turn_on_off_callback(self, req):
        """Callback for activating robot."""
        resp = SetBoolResponse()
        if not req.data:
            self._fsm = fsm.STOP
            self.stop()
            resp.success = True
            resp.message = "Robot stopped"
        else:
            if self._fsm == fsm.STOP:
                self._fsm = fsm.WAITING_FOR_LASER
                resp.success = True
                resp.message = "Robot activated"
            else:
                resp.success = False
                resp.message = "Robot already moving"

        return resp

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def random_walk(self):
        end_time = int((2*math.pi)/self.angular_velocity)
        # if change of angle was 2pi, time it takes to make
        random_num = random.randint(1, end_time)
        # rotation is 8, and we don't want to repeat so

        start_time = rospy.get_rostime()
        while (rospy.get_rostime() - start_time) < rospy.Duration(random_num):

            # initially goes counter clockwise
            if 0 < (random_num * self.angular_velocity) < math.pi:
                self.move(0, self.angular_velocity)
            else:
                self.move(0, -self.angular_velocity)

    def rotate(self):
        required_time_secs = abs(
            self.front_distance_angle/self.angular_velocity)
        start_time = rospy.get_rostime()
        while (rospy.get_rostime() - start_time) < rospy.Duration(required_time_secs):
            # if interested angle is negative and angular velocity is negative
            if self.angular_velocity < 0 and self.front_distance_angle <= 0:
                self.move(0, self.angular_velocity)
            elif self.angular_velocity < 0 and self.front_distance_angle >= 0:
                # if interested angle is positive and angular velocity is negative
                self.move(0, -self.angular_velocity)
            elif self.angular_velocity > 0 and self.front_distance_angle <= 0:
                # if interested angle is negative and angular velocity is positive
                self.move(0, -self.angular_velocity)
            else:
                # if interested angle is positive and angular velocity is positive
                self.move(0, self.angular_velocity)

    # MAPPING COMPONENTS

    def initialize_map_grid(self):
        """ Making a grid as unknown"""
        self.grid.initialize_map()  # creates the map for us

    def bresenham_line(self, x1, y1, x2, y2):  # got help from Ravi
        """ Algorithm used to convert robots sensor data to map cells
        Unknown cell in map has value of -1. This will convert things to obstacles (100), or freespace (0)"""
        deltax = x2 - x1
        deltay = y2-y1

        # if the slope > 1, then we change the increment accoriding to y, swapping x and ys
        m_steep = abs(deltay) > abs(deltax)
        if m_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        if x1 > x2:      # if x2< x1, but slope is still same, then we switch the endpoints
            x1, x2 = x2, x1
            y1, y2 = y2, y1

        # recalculating again the dx and dy
        deltax = x2 - x1
        deltay = y2-y1

        cur_y = y1
        error = 0
        y_step = -1

        if deltay >= 0:  # if slope is positive
            y_step = 1

        line_path = []

        # going through the incremental
        for cur_x in range(x1, x2+1):
            if m_steep:
                line_path.append((cur_y, cur_x))
            else:
                line_path.append((cur_x, cur_y))
            error += abs(deltay)

            if (error << 1) >= abs(deltax):
                cur_y += y_step
                error -= abs(deltax)

        return line_path

    def convert_to_map_reference(self, x, y):
        """ Given every point processed and returned by robot, it is a point in reference to odom.
        Function is used to convert these points into the map reference frame"""
        global DEFAULT
        # trans represents the translation vector, which is a 3-element tuple containing the x, y, and z

        # Map origin from odom origin in refernce to odom frame
        trans = [x_translation, y_translation, z_translation]

        # converting my euler angles to quaternion -> [0,0,0] for roll, pitch, yaw since no initial orientation change
        om_odom_initial_angles = [DEFAULT, DEFAULT, DEFAULT]

        # om_odom_initial_angles = [self.or_x, self.or_y, self.or_z]
        rot = tf.transformations.quaternion_from_euler(
            om_odom_initial_angles[0], om_odom_initial_angles[1], om_odom_initial_angles[2])

        # making my transofrmation matrix for occupancy grid in reference to odom
        t = tf.transformations.translation_matrix(trans)
        R = tf.transformations.quaternion_matrix(rot)

        o_T_om = t.dot(R)
        # odom point in reference to occupancy grid
        om_T_o = np.linalg.inv(o_T_om)

        odom_frame_vector = [x, y, DEFAULT, 1]
        odom_frame_vector = np.array(odom_frame_vector)

        # sample = [3,0,0,1]   # should return 8,5
        # sample = np.array(sample)
        # new_vector = om_T_o.dot(sample)

        map_frame_vector = om_T_o.dot(odom_frame_vector)

        # gives a new x, y in terms of occupancy grid frame
        return (map_frame_vector[0], map_frame_vector[1])
        # new vector is vector in reference to actual map/ our interested occupancy grid, so:
        # in form [x, y, z, 1]

    def create_grid_message(self):
        """ Publishing the grid we made/ updated """
        global x_translation, y_translation, z_translation
        o_grid = OccupancyGrid()

        # header stuff
        o_grid.header.frame_id = DEFAULT_ODOM_TOPIC
        o_grid.header.stamp = rospy.Time.now()

        # info stuff
        o_grid.info.resolution = self.grid.resolution
        o_grid.info.width = self.grid.rows  # reference to cells
        o_grid.info.height = self.grid.cols

        # original pose
        original_point = Point(x_translation, y_translation, z_translation)
        quaternion = Quaternion(0, 0, 0, 1)
        o_grid.info.origin = Pose(original_point, quaternion)
        grid = np.array(self.grid.grid_array)
        o_grid.data = grid.flatten()  # flattening the grid to 1D
        # Publish the occupancy grid
        self._map_meta_data_pub.publish(o_grid.info)
        self.map_publisher.publish(o_grid)

    # Sensor Data Processing

    def odom_callback(data):
        """ Processing odom's data. We actuallly use the tf listener instead, but if I ever make this
        more complicated, might be really useful"""
        pass

    def update_position(self, time_stamp):
        """ Using the tf listener to get more accurate position of robot"""
        # used base_scan previously
        self.listener.waitForTransform(
            DEFAULT_ODOM_TOPIC, 'base_laser_link', time_stamp, rospy.Duration(4.0))
        # used base_scan previously

        # Get the transform from base_link to map
        (trans, rot) = self.listener.lookupTransform(
            DEFAULT_ODOM_TOPIC, 'base_laser_link', time_stamp)

        self.p_x = trans[0]  # updating position
        self.p_y = trans[1]

        # updating orientations
        self.or_x = tf.transformations.euler_from_quaternion(rot)[0]
        self.or_y = tf.transformations.euler_from_quaternion(rot)[1]
        self.or_z = tf.transformations.euler_from_quaternion(rot)[2]  # yaw

    def map(self, lidar_scan_range, angle_min, angle_increment, time_stamp):
        # making sure I have updated positions x and y for the robot in reference to map:
        self.update_position(time_stamp)

        # convert robot's point to map reference: in meters
        (map_px, map_py) = self.convert_to_map_reference(self.p_x, self.p_y)

        # given your position in odom: and given a point in the lidar scan:
        # loop through the lidar scan
        cur_angle = angle_min

        self.grid.grid_array = np.array(self.grid.grid_array)
        for i in range(0, len(lidar_scan_range)):

            # starting with cur_angle
            distance = lidar_scan_range[i]

            # gets the point in odom reference in meters
            # (trans, rot) = self.listener.lookupTransform(DEFAULT_ODOM_TOPIC, 'base_scan', time_stamp)
            (trans, rot) = self.listener.lookupTransform(
                DEFAULT_ODOM_TOPIC, DAFAULT_BASE_LINK_TOPIC, time_stamp)
            odom_goal_x = trans[0] + distance * math.cos(cur_angle+self.or_z)
            odom_goal_y = trans[1] + distance * math.sin(cur_angle + self.or_z)

            # converts the point in map reference in meters
            (map_goal_x, map_goal_y) = self.convert_to_map_reference(
                odom_goal_x, odom_goal_y)

            # goal point rasterized
            goal_x = int(map_goal_x / self.grid.resolution)
            goal_y = int(map_goal_y / self.grid.resolution)

            # checking if the new goal x and y are within bounds:
            if self.grid.cell_exists(goal_x, goal_y):

                # initial point in map reference frame, rasterized
                intial_x = int(map_px/self.grid.resolution)
                intial_y = int(map_py/self.grid.resolution)

                # get the list for bresenheim
                path_cells = self.bresenham_line(
                    intial_x, intial_y, goal_x, goal_y)

                # the path of cells towards goal x and goal y
                for i in range(0, len(path_cells)):

                    cur_pos_x = path_cells[i][0]
                    cur_pos_y = path_cells[i][1]
                    if self.grid.cell_exists(cur_pos_x, cur_pos_y) == True:
                        # since we already set the goal as an obstacle
                        if cur_pos_x == goal_x and cur_pos_y == goal_y:
                            self.grid.set_cell(goal_x, goal_y, OBSTACLE_VAL)
                        else:
                            # if not then it is considered free space
                            self.grid.set_cell(
                                cur_pos_x, cur_pos_y, FREESPACE_VAL)
            else:
                print("not existing {} {}".format(goal_x, goal_y))
            # update to the next angle
            cur_angle += angle_increment

        # publish the grid with the updated scanning messages
        self.create_grid_message()

    # LASER CALL BACK REALLY IMPORTANT SO WE DEAL WITH THIS AT THE END

    def _laser_callback(self, msg):
        """Processing of laser message to make map"""
        """Callback to keep track and process robot's front obstacles, the right wall"""

        if len(self.grid.grid_array) == 0:
            self.initialize_map_grid()
        else:

            # mapping it out first
            self.map(msg.ranges, msg.angle_min,
                     msg.angle_increment, msg.header.stamp)
            # for right wall
            min_index = int(
                (self.scan_angle[0]-msg.angle_min) // msg.angle_increment)
            max_index = int(
                (self.scan_angle[1]-msg.angle_min) // msg.angle_increment)

            # for front wall
            min_index_front = int(
                (self.scan_angle_front[0]-msg.angle_min) // msg.angle_increment)
            max_index_front = int(
                (self.scan_angle_front[1]-msg.angle_min) // msg.angle_increment)

            i = min_index
            # for the right of the robot
            min_range_value = msg.ranges[min_index]
            while i < max_index:  # getting min value in interest range

                if msg.ranges[i] < min_range_value:
                    min_range_value = msg.ranges[i]
                i = i + 1
            # for the front of the robot
            j = min_index_front
            k = min_index_front

            # for the right of the robot
            min_range_value_front = msg.ranges[min_index_front]
            while j < max_index_front:  # getting min value in interest range

                if msg.ranges[j] < min_range_value_front:
                    k = j
                    min_range_value_front = msg.ranges[j]
                j = j + 1

            self.front_distance = min_range_value_front

            # refer to constructor on these.
            front_angle = ((min_index_front + k) *
                           msg.angle_increment) + msg.angle_min
            self.front_distance_angle = (
                self.scan_angle_front[1]-self.scan_angle_front[0]) - front_angle

            # for PD: getting min distance on robot's right and calculating error to input for PD controller to change velocity
            distance_from_wall = min_range_value
            error = self.desired_threshold_distance - distance_from_wall
            current_timestamp = msg.header.stamp
            dt = (current_timestamp - self.previous_timestamp).to_sec()

            # this is the linear
            self.u = self.pid.step(error, dt)
            # updating the next previous timestamp
            self.previous_timestamp = current_timestamp

            if self._fsm == fsm.WAITING_FOR_LASER or self._fsm == fsm.WAITING_FOR_LASER:
                if self.front_distance < 2 and min_range_value < 2:
                    self._fsm = fsm.RANDOMIZE
                elif self.front_distance < self.desired_threshold_distance_front:
                    self._fsm = fsm.ROTATE
                elif abs(error) > 0:
                    self._fsm = fsm.FOLLOW_WALL

    def spin(self):
        """Robot's actions based on finite states"""
        rate = rospy.Rate(FREQUENCY)  # loop at 10 Hz.
        while not rospy.is_shutdown():
            if self._fsm == fsm.MOVE_FORWARD:
                self.move(self.linear_velocity, 0)

            # if it should turn left
            if self._fsm == fsm.RANDOMIZE:
                self.random_walk()  # randomized rotation
                self._fsm = fsm.MOVE_FORWARD

            if self._fsm == fsm.ROTATE:
                self.rotate()  # rotation to go left
                self._fsm = fsm.WAITING_FOR_LASER

            # if it should just follow wall, no obstacles in front
            if self._fsm == fsm.FOLLOW_WALL:

                start_time = rospy.get_rostime()
                while (rospy.get_rostime() - start_time) < rospy.Duration(self.time_change):
                    # print(self.u)
                    self.move(self.linear_velocity, self.u)
                    # print("x", self.p_x, "y", self.p_y, "theta", self.or_z)

                time.sleep(self.time_change)
                self._fsm = fsm.MOVE_FORWARD

            rate.sleep()


def main():
    """Main function."""

    rospy.init_node("explorer")  # 1st. initialization of node.
    rospy.sleep(2)  # Sleep for a few seconds to wait for the registration.

    # Initialization of the class for the robot to follow the wall
    explorer1 = Explorer()

    rospy.on_shutdown(explorer1.stop)  # If interrupted, send a stop command

    try:
        explorer1.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()