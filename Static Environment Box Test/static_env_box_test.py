import math
import rclpy
import rospy
import tf
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
import tf2_ros
import tf_transformations as tft

# ... (Other necessary imports)

SPEED = 2.0  # Meters per second
LIN_TOL = 0.15  # Tolerance for reaching coordinates (meters)
ANG_TOL = 5.0  # Tolerance for reaching angles (degrees)

class BoxTest(Node):

    def __init__(self):
        super().__init__('box_test')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # ... (Initialize other variables and methods)

    def run_test(self, left_turn=True):
        # Initial movement
        self.move_to_point(2.5, 2.5, SPEED, LIN_TOL, ANG_TOL)

        # Forward motions with left turns
        for _ in range(4):
            self.move_forward(2.5, SPEED, LIN_TOL)
            self.turn(90 if left_turn else -90, SPEED, ANG_TOL)

        # Final movement and return
        self.move_forward(2.5, SPEED, LIN_TOL)
        self.turn(-90 if left_turn else 90, SPEED, ANG_TOL)
        self.move_backward(2.5, SPEED, LIN_TOL)

        # Signal program completion
        # ... (Implement your preferred signaling method)

        # Test complete
        rclpy.shutdown()

    def move_to_point(self, x, y, speed, lin_tol, ang_tol):
        """
        Moves the robot to the specified point (x, y) using Twist messages.

        Args:
            x (float): X-coordinate of the target point (meters).
            y (float): Y-coordinate of the target point (meters).
            speed (float): Desired linear speed (meters per second).
            lin_tol (float): Tolerance for reaching the target point in meters.
            ang_tol (float): Tolerance for reaching the target angle in degrees.

        Returns:
            bool: True if the target point is reached within the tolerances, False otherwise.
        """

    # Get current robot pose (x, y, theta) using odometry data or tf2_ros
        current_pose = self.get_current_pose()

        # Calculate the desired heading angle to reach the target point
        target_angle = math.atan2(y - current_pose[1], x - current_pose[0])

        # Rotate to the target angle
        if abs(target_angle - current_pose[2]) > ang_tol:
            self.turn(target_angle - current_pose[2], speed, ang_tol)

        # Move forward until the target point is reached within tolerance
        distance_to_target = math.sqrt((x - current_pose[0])**2 + (y - current_pose[1])**2)
        while distance_to_target > lin_tol:
            self.move_forward(min(distance_to_target, speed * 0.1), speed, lin_tol)  # Limit speed based on remaining distance
            # Update current pose and distance
            current_pose = self.get_current_pose()
            distance_to_target = math.sqrt((x - current_pose[0])**2 + (y - current_pose[1])**2)

        return True  # Target point reached

# Function to get the current robot pose (x, y, theta) using odometry data or tf2_ros
    def get_current_pose(self):
        # Implement logic to retrieve the robot's current pose using your chosen library
        pass


    def move_forward(self, distance, speed, lin_tol):
        """
        Moves the robot forward for a specified distance using Twist messages.

        Args:
            distance (float): Distance to move forward in meters.
            speed (float): Desired linear speed (meters per second).
            lin_tol (float): Tolerance for reaching the target distance in meters.

        Returns:
            bool: True if the target distance is reached within the tolerance, False otherwise.
        """

        # Send Twist message with forward linear velocity
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)

        # Track traveled distance and stop when target is reached


    def turn(self, angle, speed, ang_tol):
        """
        Turns the robot by a specified angle using Twist messages.

        Args:
            angle (float): Angle to turn in degrees. Positive for clockwise, negative for counter-clockwise.
            speed (float): Desired rotational speed (radians per second).
            ang_tol (float): Tolerance for reaching the target angle in degrees.

        Returns:
            bool: True if the target angle is reached within the tolerance, False otherwise.
        """

        # Get current robot heading angle (theta)
        current_theta = self.get_current_theta()

        # Calculate the required angular velocity based on the direction and target angle
        angular_velocity = (angle - current_theta) * speed / abs(angle - current_theta)

        # Send Twist message with the calculated angular velocity
        twist = Twist()
        twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist)

        # Track the rotation and stop when the target angle is reached within tolerance
        while abs(angle - current_theta) > ang_tol:
            # Use gyroscope or IMU data to get current heading angle
            current_theta = self.get_current_theta()
            # Update time and calculate remaining angle
            # ... (Implement logic to track rotation time and remaining angle)

        # Stop after reaching the target angle
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        return True  # Target angle reached

    # Function to get the current robot heading angle (theta)
    def get_current_theta(self):
        """
        Retrieves the robot's current heading angle (theta) from the base_link transform.

        Returns:
            float: Robot's current heading angle in radians, or None on error.
        """

        try:
            # Get current transform between base_link and map frames
            trans = tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
            # Extract quaternion data from the transform
            quat = trans.transform.rotation

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            euler_angles = tft.euler_from_quaternion(quat)

            # Extract yaw angle (represents heading)
            heading_angle = euler_angles[2]

            return heading_angle

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn(f"Error getting transform: {e}")
            return None


    def move_backward(self, distance, speed, lin_tol):
        """
        Moves the robot backward for a specified distance using Twist messages.

        Args:
            distance (float): Distance to move backward in meters.
            speed (float): Desired linear speed (meters per second).
            lin_tol (float): Tolerance for reaching the target distance in meters.

        Returns:
            bool: True if the target distance is reached within the tolerance, False otherwise.
        """

        # Send Twist message with backward linear velocity (negative speed)
        twist = Twist()
        twist.linear.x = -speed
        self.cmd_vel_pub.publish(twist)

        # Track traveled distance and stop when target is reached
        # Similar logic as in `move_forward` but with negative distance and speed
        # ... (Adapt the distance tracking and loop from `move_forward`)

        return True  # Target distance reached


def main(args=None):
    rclpy.init(args=args)
    node = BoxTest()

    # Run test with left turns
    node.run_test(left_turn=True)

    # Optionally run test with right turns
    # node.run_test(left_turn=False)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
