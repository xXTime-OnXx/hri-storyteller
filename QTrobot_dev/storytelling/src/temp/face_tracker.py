#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from qt_nuitrack_app.msg import Faces

class FaceTracker(object):
    def __init__(self):
        # Initialize the ROS node. Using a more descriptive name is good practice.
        rospy.init_node('face_tracker_nuitrack')

        # --- Parameters (these can stay the same) ---
        self.IMAGE_WIDTH = 640
        self.IMAGE_HEIGHT = 480
        self.KP_PAN = 0.002
        self.KP_TILT = 0.002
        self.DEAD_ZONE_X = 20
        self.DEAD_ZONE_Y = 20
        
        # --- State Variables (these can stay the same) ---
        self.current_pan = 0.0
        self.current_tilt = 0.0
        
        # --- ROS Publishers and Subscribers ---

        self.head_pub = rospy.Publisher('/qt_robot/head_controller/command', JointTrajectory, queue_size=1)
        rospy.Subscriber('/qt_nuitrack_app/faces', Faces, self.face_callback)
        rospy.Subscriber('/qt_robot/joints/state', JointTrajectory, self.joint_state_callback)

        rospy.loginfo("Face tracker node (using Nuitrack) started.")
        rospy.spin()

    def joint_state_callback(self, msg):
        try:
            pan_index = msg.joint_names.index('HeadPan')
            tilt_index = msg.joint_names.index('HeadTilt')
            self.current_pan = msg.points[0].positions[pan_index]
            self.current_tilt = msg.points[0].positions[tilt_index]
        except ValueError:
            pass

    def face_callback(self, msg):
        """This function is called every time a face is detected by Nuitrack."""
        rospy.loginfo(msg)
        if not msg.faces:
            # No faces detected, so we do nothing
            return

        # Let's just track the first face detected
        face = msg.faces[0]

        # --- CHANGED: Extract bounding box from the 'rectangle' array ---
        # The 'rectangle' array format is typically [x, y, width, height].
        # It's good practice to check its length before accessing it.
        if len(face.rectangle) < 4:
            rospy.logwarn("Received a face message with an incomplete rectangle.")
            return

        face_x = face.rectangle[0]
        face_y = face.rectangle[1]
        face_width = face.rectangle[2]
        face_height = face.rectangle[3]

        # Calculate the center of the face
        face_center_x = face_x + (face_width / 2.0)
        face_center_y = face_y + (face_height / 2.0)

        # Calculate the error (distance from the image center)
        error_pan = self.IMAGE_WIDTH / 2.0 - face_center_x
        error_tilt = self.IMAGE_HEIGHT / 2.0 - face_center_y

        # Apply dead zone to prevent jittering
        if abs(error_pan) < self.DEAD_ZONE_X:
            error_pan = 0
        if abs(error_tilt) < self.DEAD_ZONE_Y:
            error_tilt = 0
            
        if error_pan == 0 and error_tilt == 0:
            return

        # Calculate the target position
        target_pan = self.current_pan + (self.KP_PAN * error_pan)
        target_tilt = self.current_tilt - (self.KP_TILT * error_tilt) # Tilt is often inverted

        # Construct and publish the motor command
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ["HeadPan", "HeadTilt"]
        point = JointTrajectoryPoint()
        point.positions = [target_pan, target_tilt]
        point.time_from_start = rospy.Duration(0.1)
        traj_msg.points.append(point)

        self.head_pub.publish(traj_msg)

if __name__ == '__main__':
    try:
        FaceTracker()
    except rospy.ROSInterruptException:
        pass