#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from qt_nuitrack_app.msg import Faces # Using the message type you confirmed

class FaceTracker(object):
    def __init__(self):
        rospy.init_node('face_tracker_node')
        rospy.loginfo("Starting Face Tracker Node...")

        self.pan_gain = rospy.get_param('~pan_gain', -75.0)
        self.tilt_gain = rospy.get_param('~tilt_gain', 40.0)

        self.center_x = 0.5
        self.center_y = 0.5
        
        self.pan_min = rospy.get_param('~pan_min', -40.0) # Min Yaw (left/right)
        self.pan_max = rospy.get_param('~pan_max', 40.0) # Max Yaw (left/right)
        
        self.tilt_min = rospy.get_param('~tilt_min', -25.0) # Min Pitch (down)
        self.tilt_max = rospy.get_param('~tilt_max', 15.0)  # Max Pitch (up)

        # === Publisher ===
        self.head_pub = rospy.Publisher('/qt_robot/head_position/command', 
                                        Float64MultiArray, 
                                        queue_size=1)

        # === Subscriber ===
        self.face_sub = rospy.Subscriber('/qt_nuitrack_app/faces',
                                         Faces, 
                                         self.face_callback)
        
        rospy.loginfo("Tracker initialized. Waiting for 2D face data...")

    def face_callback(self, msg):
        if not msg.faces:
            return

        face = msg.faces[0]

        try:
            # Check if eye data is available and is 2D
            if not face.left_eye or not face.right_eye or len(face.left_eye) < 2 or len(face.right_eye) < 2:
                rospy.logwarn_once("Eye data not available or not 2D.")
                return

            # Calculate the 2D midpoint between the eyes
            eye_mid_x = (face.left_eye[0] + face.right_eye[0]) / 2.0
            eye_mid_y = (face.left_eye[1] + face.right_eye[1]) / 2.0
            
            # Calculate the error from the center of the screen
            error_x = eye_mid_x - self.center_x
            error_y = eye_mid_y - self.center_y
            
        except Exception as e:
            rospy.logerr("Error processing 2D eye data: {}".format(e))
            return
            
        # --- This is the P-Controller Logic ---
        pan_command = self.pan_gain * error_x
        tilt_command = self.tilt_gain * error_y

        # --- Clamping ---
        pan_command = max(self.pan_min, min(self.pan_max, pan_command))
        tilt_command = max(self.tilt_min, min(self.tilt_max, tilt_command))

        # --- Publish the Command ---
        head_msg = Float64MultiArray()
        head_msg.data = [pan_command, tilt_command]
        
        #rospy.loginfo("Target at [x:{:.2f}, y:{:.2f}], Error [x:{:.2f}, y:{:.2f}], Command [Pan:{:.2f}, Tilt:{:.2f}]".format(
        #    eye_mid_x, eye_mid_y, error_x, error_y, pan_command, tilt_command
        #))
        
        self.head_pub.publish(head_msg)


if __name__ == '__main__':
    try:
        FaceTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Face tracker node shut down.")