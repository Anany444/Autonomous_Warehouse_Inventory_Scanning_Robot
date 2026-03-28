import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from action_msgs.msg import GoalStatus
from warehouse_msgs.msg import RackArray, Rack
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import math

SERVER_WAIT_TIMEOUT_SEC = 5.0

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation')
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)
        
        self.rack_subscription = self.create_subscription(
            RackArray,
            '/detected_racks',
            self.rack_callback,
            10)
    
        self.racks_found_service = self.create_service(Trigger, '/racks_found', self.racks_found_callback)
        self.start_nav_service = self.create_service(Trigger, '/start_navigation', self.start_navigation_callback)
        self.start_scan_service_client = self.create_client(Trigger, '/start_scan')
        self.stop_scan_service_client = self.create_client(Trigger, '/stop_scan')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.action_client_through_poses = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self.action_client_z_controller = ActionClient(self, FollowJointTrajectory, '/z_controller/follow_joint_trajectory')
        self.timer = self.create_timer(1.0, self.navigation_and_scanning)
        
        self.rack_positions_in_map_coord = []
        self.rack_positions_in_world_coord = []
        self.capture_positions_in_world_coord = []
        self.capture_positions_in_world_coord_ordered = []
        self.curr_rack_array = None
        
        self.target_rack_index = 0
        self.target_scan_index = 0
        self.nav_state = ""
        self.found_racks = False
        self.curr_z_target = 1.4
        
        self.simple_map_curr = None
        self.time_start_once_bool = True
        self.time_start = None
        self.goal_completed = True
        self.goal_handle_curr= None
        self.goal_accepted = False 
        
        self.logger=self.get_logger()

    def map_callback(self, msg: OccupancyGrid):
        self.simple_map_curr = msg
    def odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        #self.get_logger().info('Robot position - x: {:.2f}, y: {:.2f}'.format(position.x, position.y))
    
    def racks_found_callback(self, request, response):
        self.found_racks = True
        self.rack_positions_in_map_coord = [(rack.x, rack.y, rack.theta_deg) for rack in self.curr_rack_array.racks]
        for (cx, cy, theta) in self.rack_positions_in_map_coord:
            wx,wy = self.get_world_coord_from_map_coord(cx, cy)
            self.rack_positions_in_world_coord.append((wx, wy, theta))
            px,py, angle = self.find_capture_position_near_rack(wx, wy, theta)
            self.capture_positions_in_world_coord.append((px, py, angle))
        self.capture_positions_in_world_coord_ordered = sorted(self.capture_positions_in_world_coord, key=lambda p: math.hypot(p[0], p[1]))
        response.success = True
        response.message = f'Processed {len(self.rack_positions_in_map_coord)} racks, generated {len(self.capture_positions_in_world_coord_ordered)} capture positions.'
        return response
    
    def start_navigation_callback(self, request, response):
        self.nav_state = "NAVIGATING"
        response.success = True
        response.message = 'Navigation started.'
        return response
        
    def rack_callback(self, msg: RackArray):
        self.curr_rack_array = msg
        if self.found_racks:
            return
        
    def navigation_and_scanning(self):
        if self.nav_state != "NAVIGATING" and self.nav_state != "SCANNING":
            return
        if not self.goal_completed :
            return
        if self.target_rack_index > self.target_scan_index and self.nav_state != "SCANNING":
            self.nav_state = "SCANNING"
            self.start_scan_service_client.wait_for_service(timeout_sec=SERVER_WAIT_TIMEOUT_SEC)
            self.start_scan_service_client.call_async(Trigger.Request())
            self.send_z_controller_goal()
            return
            
        if self.target_rack_index >= len(self.capture_positions_in_world_coord_ordered):
            self.logger.info("All racks have been navigated to!")
            self.nav_state = "COMPLETED"
            return
        
        if self.nav_state != "NAVIGATING":
            return
        
        target_capture_pos = self.capture_positions_in_world_coord_ordered[self.target_rack_index]
        goal_pose = self.create_goal_pose(target_capture_pos[0], target_capture_pos[1], target_capture_pos[2])
        if self.send_goal_from_world_pose(goal_pose):
            self.logger.info(f"Sent goal for rack {self.target_rack_index+1} at capture position (x={target_capture_pos[0]:.2f}, y={target_capture_pos[1]:.2f}, angle={target_capture_pos[2]:.2f} deg)")
            self.target_rack_index += 1
        
        else:
            self.logger.warn(f"Failed to send goal for rack {self.target_rack_index+1} at capture position (x={target_capture_pos[0]:.2f}, y={target_capture_pos[1]:.2f}, angle={target_capture_pos[2]:.2f} deg). Will retry.")
        
    def send_z_controller_goal(self):
        self.action_client_z_controller.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT_SEC)
        
        # Create trajectory
        traj = JointTrajectory()
        traj.joint_names = ['camera_joint']

        point = JointTrajectoryPoint()
        point.positions = [self.curr_z_target]          # target position
        point.time_from_start.sec = 10   # duration

        traj.points.append(point)

        # Create goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj
        self.curr_z_target = 1.4 if self.curr_z_target < 1.0 else 0.1
        # Send goal (no feedback handling)
        goal_future = self.action_client_z_controller.send_goal_async(goal_msg)
        goal_future.add_done_callback(self.z_controller_response_callback)
        self.get_logger().info("Goal sent!")
        
    def z_controller_response_callback(self, future):
        self.get_logger().info("Z controller goal acceptd!")
        
        
        result_future = future.result().get_result_async()
        result_future.add_done_callback(self.z_controller_result_callback)
        
    def z_controller_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Z controller action result: {result}")
        if result.error_code == 0:  # SUCCESS
            self.get_logger().info("Z controller action succeeded!")
            self.target_scan_index += 1
            self.stop_scan_service_client.wait_for_service(timeout_sec=SERVER_WAIT_TIMEOUT_SEC)
            self.stop_scan_service_client.call_async(Trigger.Request())
            self.nav_state = "NAVIGATING"
        else:
            self.get_logger().warn(f"Z controller action failed with status: {result}")
        
        
    def create_goal_pose(self, x, y, yaw_deg):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation = self._create_quaternion_from_yaw(math.radians(yaw_deg))
        return goal_pose
    
    def create_goal_through_poses(self, waypoints):
        goal = NavigateThroughPoses.Goal()
        goal.poses = []
        for (x, y, yaw_deg) in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.orientation = self._create_quaternion_from_yaw(math.radians(yaw_deg))
            goal.poses.append(pose_stamped)
        return goal
        
    def send_goal_from_world_pose(self, goal_pose):
        if not self.goal_completed or self.goal_handle_curr is not None:
            return False

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        if not self.action_client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT_SEC):
            self.logger.error('NavigateToPose action server not available!')
            return False
        
        self.goal_completed = False  # Starting a new goal.

        # Send goal asynchronously (non-blocking).
        goal_future = self.action_client.send_goal_async(goal, self.goal_feedback_callback)
        goal_future.add_done_callback(self.goal_response_callback)
        self.logger.info(f'Goal sent : x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}')
        return True
    
    def send_goal_through_poses_from_world_pose(self, waypoints):
        if not self.goal_completed or self.goal_handle_curr is not None:
            return False
                
        goal = NavigateThroughPoses.Goal()
        goal.poses = waypoints
        
        if not self.action_client_through_poses.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT_SEC):
            self.logger.error('NavigateThroughPoses action server not available!')
            return False

        self.goal_completed = False  # Starting a new goal.

        # Send goal asynchronously (non-blocking).
        goal_future = self.action_client_through_poses.send_goal_async(goal, self.goal_feedback_callback)
        goal_future.add_done_callback(self.goal_response_callback)
        self.logger.info(f'Goal through poses sent with {len(waypoints)} waypoints.')
        return True
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.warn('Goal rejected :(')
            self.goal_completed = True  # Mark goal as completed (rejected).
            self.goal_handle_curr = None  # Clear goal handle.
        else:
            self.logger.info('Goal accepted :)')
            
            self.goal_accepted = True 
            self.goal_completed = False  # Mark goal as in progress.
            self.goal_handle_curr = goal_handle  # Store goal handle.

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            
            self.logger.info("Goal completed successfully!")
            
        else:
            self.logger.warn(f"Goal failed with status: {status}")
        self.goal_completed = True  # Mark goal as completed.
        self.goal_handle_curr = None  # Clear goal handle
    
    def goal_feedback_callback(self, msg):
        distance_remaining = msg.feedback.distance_remaining
        navigation_time = msg.feedback.navigation_time.sec
        estimated_time_remaining = msg.feedback.estimated_time_remaining.sec

        self.logger.debug(f"Navigation time: {navigation_time}s, "
                  f"Distance remaining: {distance_remaining:.2f}, "
                  f"Estimated time remaining: {estimated_time_remaining}s")

    def get_world_coord_from_map_coord(self, map_x, map_y):
        map_info = self.simple_map_curr
        if map_info:
            resolution, origin_x, origin_y = map_info.info.resolution, map_info.info.origin.position.x, map_info.info.origin.position.y
            world_x = (map_x + 0.5) * resolution + origin_x
            world_y = (map_y + 0.5) * resolution + origin_y
            return (world_x, world_y)
        else:
            return (0.0, 0.0)
        
    def _create_quaternion_from_yaw(self, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = sy
        q.w = cy
        return q
    
    def find_capture_position_near_rack(self, rack_world_x, rack_world_y,rack_orientation_deg, offset_distance=0.6, ):
        # Calculate a position offset from the rack position
        capture_x = rack_world_x + offset_distance*math.sin(math.radians(rack_orientation_deg))
        capture_y = rack_world_y + offset_distance*math.cos(math.radians(rack_orientation_deg))
        if abs(rack_orientation_deg-90.0)< 10.0:
            rack_orientation_deg-=180.0
        return (capture_x, capture_y, rack_orientation_deg) 
                
def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()    
    
if __name__ == '__main__':
    main()