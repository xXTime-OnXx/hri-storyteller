import rospy
from std_msgs.msg import Float64MultiArray
 
class HeadController:
    def __init__(self):
        # Publisher
        self.head_pub = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=1)
       
        # --- CONFIGURATION ---
        # Adjust these based on your camera resolution
        # If your 'x' and 'w' are percentages (0-100), set WIDTH=100.
        # If they are pixels (e.g. VGA), set WIDTH=640.
        self.IMG_WIDTH = 640  
        self.IMG_HEIGHT = 480
       
        # Safety Limits (Degrees)
        self.MAX_YAW = 45.0
        self.MIN_YAW = -45.0
        self.MAX_PITCH = 10.0   # Looking down (usually positive on QT)
        self.MIN_PITCH = -20.0  # Looking up
 
        # Gains (How fast it moves)
        # If it oscillates (shakes head), lower these numbers.
        # If it's too slow, increase them.
        self.KP_X = 0.01
        self.KP_Y = 0.01
 
        # Internal State (Where we think the head is)
        self.current_yaw = 0.0
        self.current_pitch = 0.0
 
    def track_face(self, face_rect):
        """
        Input: region dictionary {'x': 2, 'y': 2, 'w': 70, 'h': 70}
        """
       
        # 1. Calculate Face Center
        face_cx = face_rect['x'] + (face_rect['w'] / 2)
        face_cy = face_rect['y'] + (face_rect['h'] / 2)
 
        # 2. Calculate Error (Distance from image center)
        # Center of image
        img_cx = self.IMG_WIDTH / 2
        img_cy = self.IMG_HEIGHT / 2
 
        # Error: How far is face from center?
        error_x = img_cx - face_cx
        error_y = img_cy - face_cy
 
        # 3. Calculate Delta (How much to move)
        # Note: Directions depend on robot mounting.
        # Usually: Face on Left (x < cx) -> Error Positive -> Turn Left (Positive Yaw)
        delta_yaw = self.KP_X * error_x
        delta_pitch = self.KP_Y * error_y # Check sign: Face Up (y < cy) -> Error Positive -> Look Up (Negative Pitch?)
 
        # 4. Update Current State
        self.current_yaw += delta_yaw
        # Pitch often needs inverted sign depending on robot.
        # If face is UP (y small), error_y is positive. We need Pitch to go NEGATIVE (Look Up).
        self.current_pitch -= delta_pitch
 
        # 5. Clamp Values (Safety)
        self.current_yaw = max(min(self.current_yaw, self.MAX_YAW), self.MIN_YAW)
        self.current_pitch = max(min(self.current_pitch, self.MAX_PITCH), self.MIN_PITCH)
 
        # 6. Publish
        self.publish_command()
 
    def publish_command(self):
        msg = Float64MultiArray()
        msg.data = [self.current_yaw, self.current_pitch]
        self.head_pub.publish(msg)