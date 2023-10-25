#!/usr/bin/env python

# Add Offboard control > Done
# Add optical avoidance? > X

import rospy
import tf.transformations
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
from std_msgs.msg import Empty
from actionlib_msgs.msg import GoalID
from pymavlink import mavutil
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool,CommandBoolRequest, SetModeRequest


class AutonomousMapping:
    def __init__(self):
        rospy.init_node('autonomous_mapping')

        # current state
        self.current_state = State()

        # Drone state
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_callback)

        # MAVLink Initialization
        self.initialize_mavlink()

        # Starting Drone armed and offboard after Takeoff
        self.start()

        # Init takeoff 2M Check
        self.has_reached_initial_altitude = False

        # ROS Variables and Parameters
        self.current_map = None
        self.current_pose = None
        self.mapping_completed = False
        self.move_base_path = None
        self.last_sent_target_pose = Pose()

        # Frame id setting
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        self.global_frame_id = rospy.get_param('~global_frame_id', 'map')

        # ROS Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.occupancy_grid_callback, queue_size=10)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_callback)

        rospy.Subscriber('/move_base/local_costmap', OccupancyGrid, self.local_costmap_callback)

        # ROS Publishers
        self.pub_setpoint = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.pub_done = rospy.Publisher('mapping_done', Empty, queue_size=1)
        self.cancel_move_base_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

        # Move-base
        self.move_base_client = SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()


    def state_callback(self, msg):
        self.current_state = msg

    def start(self):

        local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        rate = rospy.Rate(20)

        while (not rospy.is_shutdown() and not self.current_state.connected):
            rate.sleep()

        pose = PoseStamped()

        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 2

        for i in range(100):
            if (rospy.is_shutdown()):
                break

            local_pos_pub.publish(pose)
            rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()

            # Drone Armed
            if not self.current_state.armed and (now - last_req > rospy.Duration(5.0)):
                if arming_client(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")
                last_req = now

            # OFFBOARD Mode setting
            elif self.current_state.mode != "OFFBOARD" and (now - last_req > rospy.Duration(5.0)):
                if set_mode_client(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")
                last_req = now

            # Cheking armed, OFFBOARD Mode
            if self.current_state.armed and self.current_state.mode == "OFFBOARD":
                rospy.loginfo("OFFBOARD mode set and vehicle armed. Exiting start function.")
                break

            # Send Setpoints for OFFBOARD Mode conversion
            local_pos_pub.publish(pose)
            rate.sleep()
        rospy.loginfo("OFFBOARD mode set, exiting script.")

    def initialize_mavlink(self):
        ''' Conneting Drone to MAVROS: USE HeartBeat msg to Checking connection'''
        try:
            self.master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
            msg = self.master.wait_heartbeat(timeout=10)

            if not msg:
                rospy.logerr("Timed out waiting for a HEARTBEAT message.")
                return

            if msg.type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
                rospy.loginfo("Successfully connected to the Pixhawk drone!")
            else:
                rospy.logwarn("The connected device doesn't seem to be a Pixhawk drone. Device type: %s", msg.type)
                # PX4 Drone Checking Code

        except Exception as e:
            rospy.logerr("Failed to connect to the drone via MAVLink. Error: %s", str(e))

    def check_reached_initial_altitude(self):
        # takeoff Checking 1.9M
        if self.current_pose and self.current_pose.position.z >= 1.4: # 1.9M
            self.has_reached_initial_altitude = True

    def send_takeoff_command(self):
        setpoint = PoseStamped()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = self.global_frame_id
        setpoint.pose.position.x = 0
        setpoint.pose.position.y = 0
        setpoint.pose.position.z = 1.5  # Init Takeoff setpoint User can change. 2M
        setpoint.pose.orientation = Quaternion(0, 0, 0, 1)
        self.pub_setpoint.publish(setpoint)
        rospy.loginfo("Sent takeoff command. Target altitude: 1.5M")

    def path_callback(self, msg):
        """Callback for the move_base path."""
        self.move_base_path = msg.poses
        if self.move_base_path:
            self.follow_path(self.move_base_path)

    def local_costmap_callback(self, msg):
        """ local costmap  """
        self.local_costmap = msg

    def follow_path(self, path):
        """ Send the setpoints in the path one by one. """

        """ To keep the offboard mode, you must continue to deliver coordinates to the mavros using the setpoint command. 
        In order to move the path using the parameters of the move_base,
        the path planned by the move_base can be delivered one by one to the mavros through setpoint. """

        for pose_stamped in path:
            # Send each pose as a setpoint
            target_pose = pose_stamped.pose

            # Keep sending the target pose as a setpoint until the drone reaches it or ROS shuts down
            while not self.has_reached_waypoint(target_pose) and not rospy.is_shutdown():
                self.send_target_position_to_drone(target_pose)
                rospy.sleep(1)  # Sleep for a short duration before checking again

            # Once the drone reaches the setpoint, move to the next one
            if rospy.is_shutdown():
                return

    def position_callback(self, msg):
        """Update drone's current position."""
        self.current_pose = msg.pose

    def cancel_move_base(self):
        """Cancel the current goal of move_base."""
        # YOLO's object detection can be used to cancel and reroute existing autonomous driving for tracking commands
        self.cancel_move_base_pub.publish(GoalID())

    def has_reached_waypoint(self, target_pose):
        """Check if the drone has reached the specified waypoint."""
        if not self.current_pose or not target_pose:
            return False

        # Adjust this line:
        distance = ((self.current_pose.position.x - target_pose.position.x) ** 2 +
                    (self.current_pose.position.y - target_pose.position.y) ** 2) ** 0.5

        # Consider the drone to have reached the waypoint if distance is less than a threshold (e.g., 0.5 meters)
        return distance < 0.15

    def send_target_position_to_drone(self, target_pose):
        """Send a target position to the drone to move to."""
        setpoint = PoseStamped()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = self.global_frame_id
        setpoint.pose.position.x = target_pose.position.x
        setpoint.pose.position.y = target_pose.position.y
        setpoint.pose.position.z = 1.5  # Set the altitude to meters >> To make changes so that users can make changes
        setpoint.pose.orientation = target_pose.orientation

        # Set limite orientation
        limited_orientation = self.limit_orientation(target_pose.orientation)
        setpoint.pose.orientation = limited_orientation

        self.pub_setpoint.publish(setpoint)
        rospy.loginfo("Drone setpoint: x = %f, y = %f, z = 2", target_pose.position.x, target_pose.position.y)

    def limit_orientation(self, quaternion):

        euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        max_pitch = rospy.get_param('~max_pitch', 0.1)  # Radian 0.1: 5.729
        max_roll = rospy.get_param('~max_roll', 0.1)  # Radian 0.1: 5.729

        pitch = max(-max_pitch, min(euler[1], max_pitch))
        roll = max(-max_roll, min(euler[0], max_roll))
        limited_quaternion = tf.transformations.quaternion_from_euler(roll, pitch, euler[2])

        return Quaternion(*limited_quaternion)

    def send_current_position_as_setpoint(self, event=None):
        """ Once the drone's current location has been identified, the command is passed to the mavros to fix the current location """
        if not self.current_pose:
            return

        setpoint = PoseStamped()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = self.global_frame_id
        setpoint.pose = self.current_pose

        self.pub_setpoint.publish(setpoint)


    def is_mapping_completed(self):
        """ Mapped check algorithm using the percentage of cells undetected in the entire map and thresholds using the grid """
        if not self.current_map:
            return False

        grid_data = self.current_map.data
        unexplored_count = grid_data.count(-1)
        total_cells = len(grid_data)
        unexplored_ratio = float(unexplored_count) / total_cells
        mapping_completion_threshold = 0.05

        if unexplored_ratio <= mapping_completion_threshold and not self.mapping_completed:
            self.pub_done.publish(Empty())
            self.mapping_completed = True
            rospy.loginfo("Mapping is completed.")
        return self.mapping_completed



    def occupancy_grid_callback(self, msg):
        self.current_map = msg
        rospy.loginfo("Received a map update!")

        if not hasattr(self, 'move_base_client'):
            rospy.logwarn("move_base_client is not initialized yet. Skipping this map update.")
            return

        if not self.has_reached_initial_altitude:
            # If the drone has not reached 2m altitude, do not execute the frontier exploration yet
            rospy.loginfo("Waiting for the drone to reach 2m altitude...")
            return

        if not self.has_reached_waypoint(self.last_sent_target_pose):
            # If drone has not reached the last setpoint, keep sending the same setpoint
            self.send_target_position_to_drone(self.last_sent_target_pose)
            return

        target_pose = self.frontier_exploration()
        if target_pose:
            move_base_goal = MoveBaseGoal()
            move_base_goal.target_pose = target_pose
            self.move_base_client.send_goal(move_base_goal)
            rospy.loginfo("Grid sent setpoint: x = %f, y = %f", target_pose.pose.position.x, target_pose.pose.position.y)
            self.last_sent_target_pose = target_pose.pose  # Save this target pose
            self.move_base_client.wait_for_result()
            if self.has_reached_waypoint(target_pose.pose):
                rospy.loginfo("Reached target position.")
            else:
                rospy.loginfo("Failed to reach the target position.")
        else:
            rospy.loginfo("No target position found!")
            self.last_sent_target_pose = None
            return

    def frontier_exploration(self):
        grid = self.current_map.data
        width = self.current_map.info.width
        height = self.current_map.info.height
        resolution = self.current_map.info.resolution
        origin_x, origin_y = self.current_map.info.origin.position.x, self.current_map.info.origin.position.y

        rospy.loginfo("Exploring from current position x = %f, y = %f", self.current_pose.position.x,
                      self.current_pose.position.y)

        unexplored_cells_with_neighbors = []

        for y in range(height):
            for x in range(width):
                index = y * width + x
                if grid[index] == -1:  # Unexplored
                    neighbors = self.get_eight_neighbors(x, y, width, height)
                    unexplored_neighbors = [(nx, ny) for nx, ny in neighbors if grid[ny * width + nx] == -1]
                    unexplored_cells_with_neighbors.append({'cell': (x, y), 'neighbors': unexplored_neighbors})

        unexplored_cells_with_neighbors.sort(key=lambda x: len(x['neighbors']))

        for potential_target in unexplored_cells_with_neighbors:
            target_x, target_y = potential_target['cell']
            distance_to_target = ((self.current_pose.position.x - target_x) ** 2 +
                                  (self.current_pose.position.y - target_y) ** 2) ** 0.5

            # Add a safety check: if the target is too close to the current position, skip it
            # If the LiDAR sensor measurement distance is reduced to less than 30cm based on the LiDAR sensor of the drone,
            # cancel the target coordinate, call the mapping data topic, create a new target coordinate, and move it

            if distance_to_target > 1.0:  # This threshold can be adjusted
                target_pose = PoseStamped()
                target_pose.header.frame_id = "map"
                target_pose.pose.position = Point(target_x * resolution + origin_x, target_y * resolution + origin_y,
                                                  1.5)  # Set the altitude to 2.5 meters
                target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
                return target_pose

        rospy.logwarn("No suitable frontier found!")
        return None

    def get_eight_neighbors(self, x, y, width, height):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    neighbors.append((nx, ny))
        return neighbors

    def run(self):
        rate = rospy.Rate(20)  # 20Hz

        while not rospy.is_shutdown():
            if not self.current_state.connected:
                rospy.logwarn("Drone is not connected yet. Waiting for connection...")
                rate.sleep()
                continue
            if self.current_state.connected:
                break

        # Drone init TAKEOFF Command
        while not self.has_reached_initial_altitude and not rospy.is_shutdown():
            self.send_takeoff_command()
            self.check_reached_initial_altitude()
            rate.sleep()

            if self.has_reached_initial_altitude:
                print("Takeoff Command Loop End")
                break


        rospy.spin()

if __name__ == '__main__':
    try:
        node = AutonomousMapping()
        node.run()
    except rospy.ROSInterruptException:
        pass

