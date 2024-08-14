import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import apriltag

class WebcamAndAprilTagNode(Node):
    def __init__(self):
        super().__init__('webcam_and_april_tag_node')
        self.bridge = CvBridge()

        # Initialize webcam
        self.cap = cv2.VideoCapture(0)  # 0 is the ID of the default webcam

        # Check if the webcam is opened correctly
        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open webcam.")
            exit()

        # Initialize AprilTag detector
        self.detector = apriltag.Detector()
        
        # Camera parameters
        self.tagsize = 0.174  # Tag size in meters
        self.fx = 530.6055215511977  # Updated focal length in x-direction
        self.fy = 535.4232524296198  # Updated focal length in y-direction
        self.cx = 321.0874788178917  # Updated principal point x-coordinate
        self.cy = 239.3952942945455  # Updated principal point y-coordinate

        # ROS publishers
        self.image_publisher = self.create_publisher(Image, 'camera_image', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'april_tag_pose', 1)
        self.camera_pose_publisher = self.create_publisher(PoseStamped, 'camera_pose', 10)

        # ROS subscriber
        self.create_subscription(Image, 'camera_image', self.image_callback, 10)

        # Start publishing images
        self.timer = self.create_timer(0.1, self.publish_image)

    def publish_image(self):
        ret, frame = self.cap.read()

        if ret:
            # Resize the frame to 160x120 pixels
            frame_resized = cv2.resize(frame, (640, 480))

            # Convert the frame to grayscale
            gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)

            # Detect AprilTags in the frame
            detections = self.detector.detect(gray)

            for detection in detections:
                corners = detection.corners
                for i in range(4):
                    start_point = tuple(map(int, corners[i]))
                    end_point = tuple(map(int, corners[(i + 1) % 4]))
                    cv2.line(frame_resized, start_point, end_point, (0, 255, 0), 2)
                
                # Draw the ID of the AprilTag
                tag_id = str(detection.tag_id)
                center_x = int((corners[0][0] + corners[2][0]) / 2)
                center_y = int((corners[0][1] + corners[2][1]) / 2)
                #cv2.putText(frame_resized, tag_id, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            #cv2.imshow('Webcam', frame_resized)
            #cv2.waitKey(1)

            # Convert the resized frame to a ROS image message
            ros_image = self.bridge.cv2_to_imgmsg(frame_resized, encoding='bgr8')

            # Set the timestamp and frame ID
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_frame'

            # Publish the image
            self.image_publisher.publish(ros_image)
        else:
            self.get_logger().error("Error: Could not read frame from webcam.")

    def image_callback(self, msg):
        self.get_logger().info("Image callback triggered")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags in the frame
        detections = self.detector.detect(gray)

        for detection in detections:
            corners = detection.corners
            object_points = np.array([
                [-self.tagsize / 2, -self.tagsize / 2, 0],
                [ self.tagsize / 2, -self.tagsize / 2, 0],
                [ self.tagsize / 2,  self.tagsize / 2, 0],
                [-self.tagsize / 2,  self.tagsize / 2, 0]
            ])
            image_points = np.array(corners)

            camera_matrix = np.array([
                [self.fx, 0, self.cx],
                [0, self.fy, self.cy],
                [0, 0, 1]
            ])
            dist_coeffs = np.zeros(4)

            retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

            if retval:
                self.get_logger().info(f"solvePnP successful for tag ID {detection.tag_id}")
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = f'tag_ID_{detection.tag_id}'  # Including ID in the frame_id
                pose_msg.pose.position.x = tvec[0][0]
                pose_msg.pose.position.y = tvec[1][0]
                pose_msg.pose.position.z = tvec[2][0]

                # Use the first three components of rvec for orientation
                pose_msg.pose.orientation.x = rvec[0][0]
                pose_msg.pose.orientation.y = rvec[1][0]
                pose_msg.pose.orientation.z = rvec[2][0]
                pose_msg.pose.orientation.w = 1.0  # Setting w component manually since solvePnP only returns a rotation vector

                self.pose_publisher.publish(pose_msg)
                print(pose_msg)

                # Inverse transformation to find the camera pose relative to the AprilTag
                R, _ = cv2.Rodrigues(rvec)
                R_inv = R.T
                tvec_inv = -R_inv @ tvec

                camera_pose_msg = PoseStamped()
                camera_pose_msg.header.stamp = self.get_clock().now().to_msg()
                camera_pose_msg.header.frame_id = f'camera_frame_relative_to_tag_{detection.tag_id}'
                camera_pose_msg.pose.position.x = tvec_inv[0][0]
                camera_pose_msg.pose.position.y = tvec_inv[1][0]
                camera_pose_msg.pose.position.z = tvec_inv[2][0]

                # Convert rotation matrix to quaternion
                quat = self.rotation_matrix_to_quaternion(R_inv)
                camera_pose_msg.pose.orientation.x = quat[0]
                camera_pose_msg.pose.orientation.y = quat[1]
                camera_pose_msg.pose.orientation.z = quat[2]
                camera_pose_msg.pose.orientation.w = quat[3]

                self.camera_pose_publisher.publish(camera_pose_msg)
            else:
                self.get_logger().error("solvePnP failed")
                self.camera_pose_publisher.publish(PoseStamped())
    def rotation_matrix_to_quaternion(self, R):
        """Convert a rotation matrix to a quaternion."""
        q = np.empty((4, ))
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s
        return q
    def yaw_pitch_roll(self,qw,qx,qy,qz):
        """Get the equivalent yaw-pitch-roll angles aka. intrinsic Tait-Bryan angles following the z-y'-x'' convention

        Returns:
            yaw:    rotation angle around the z-axis in radians, in the range `[-pi, pi]`
            pitch:  rotation angle around the y'-axis in radians, in the range `[-pi/2, pi/2]`
            roll:   rotation angle around the x''-axis in radians, in the range `[-pi, pi]` 

        The resulting rotation_matrix would be R = R_x(roll) R_y(pitch) R_z(yaw)

        Note: 
            This feature only makes sense when referring to a unit quaternion. Calling this method will implicitly normalise the Quaternion object to a unit quaternion if it is not already one.
        """
        self._normalise()



        print(2*(qx*qz-qw*qy), self.q)
        if 2*(qx*qz-qw*qy)>=0.94: #Preventing gimbal lock for north pole
            yaw = np.arctan2(qx*qy-qw*qz,qx*qz+qw*qy)
            roll = 0
        elif 2*(qx*qz-qw*qy)<=-0.94: #Preventing gimbal lock for south pole
            yaw = -np.arctan2(qx*qy-qw*qz,qx*qz+qw*qy)
            roll = 0
        else:
            yaw = np.arctan2(qy*qz + qw*qx,
                1/2 - (qx**2 + qy**2))
            roll = np.arctan2(qx*qy - qw*qz,
                1/2 - (qy**2 + qz**2))
        pitch = np.arcsin(-2*(qx * qz - qw * qy))

        return yaw, pitch, roll  
def main(args=None):
    rclpy.init(args=args)

    webcam_and_april_tag_node = WebcamAndAprilTagNode()

    rclpy.spin(webcam_and_april_tag_node)

    webcam_and_april_tag_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
