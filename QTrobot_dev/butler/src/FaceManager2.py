import rospy
import json
import os
import cv2
import numpy as np
import face_recognition
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from qt_nuitrack_app.msg import Faces, FaceInfo # Check your specific message type
from Person import Person

class FaceManager:

    def __init__(self, database_path="butler_memory.json"):
        # 1. State Management
        self.faces = {}  # Stores current active Person objects { nuitrack_id: Person }
        self.database_path = database_path
        
        # 2. Memory (The "Guest Book")
        self.known_encodings = []
        self.known_names = []
        self._load_database()

        # 3. Vision Tools
        self.bridge = CvBridge()
        self.latest_image = None
        
        # 4. Subscribers (The "Senses")
        
        # A. Camera (For recognition)
        self.sub_img = rospy.Subscriber(
            "/camera/color/image_raw", 
            Image, 
            self.image_callback
        )
        
        # B. Nuitrack (For skeleton/face tracking)
        self.sub_nuitrack = rospy.Subscriber(
            "/qt_nuitrack_app/faces", 
            Faces, 
            self.nuitrack_callback
        )

        rospy.loginfo("FaceManager initialized with hybrid Nuitrack + FaceRecognition")

    # --- CALLBACKS ---

    def image_callback(self, msg):
        try:
            # Save the latest frame for cropping later
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr_throttle(5, f"CV Bridge fail: {e}")

    def nuitrack_callback(self, msg):
        """
        Runs at ~30Hz. Updates positions instantly.
        Triggers recognition only if identity is unknown.
        """
        current_frame_faces = {}
        
        now = rospy.Time.now()

        for face in msg.faces:
            fid = face.id
            
            # 1. RETRIEVE OR CREATE PERSON
            # If we saw this ID in the last frame, keep their Name/Identity
            if fid in self.faces:
                p = self.faces[fid]
            else:
                # New ID found! Create new Person
                
                p = Person(
                    id=fid,
                    gender=face.gender,
                    emotion="neutral", # Default until updated
                    region={'x':0, 'y':0, 'w':0, 'h':0}
                )


            p.last_seen = now

            # 2. UPDATE DYNAMIC FIELDS (Position is critical for Head Control)
            # Map Nuitrack normalized/pixel rect to your Person.region format
            # Nuitrack msg usually has face.rectangle.x, etc.
            p.region = {
                'x': int(face.rectangle[0] * 640), # Assuming normalized 0-1. If pixels, remove *640
                'y': int(face.rectangle[1] * 480),
                'w': int(face.rectangle[2] * 640),
                'h': int(face.rectangle[3] * 480)
            }
            
            # Update Emotion if available
            emotions = [msg.faces[0].emotion_angry, msg.faces[0].emotion_happy, msg.faces[0].emotion_surprise]
            em = max(emotions)
            em_index = emotions.index(em)
            if em_index == 0 and em >= 0.6:
                p.emotion = "sad"
            elif em_index == 1 and em >= 0.8:
                p.emotion = "happy"
            elif em_index == 2 and em >= 0.8:
                p.emotion = "surprised"

            # 3. IDENTITY CHECK (The "Crop & Tag" Logic)
            if not p.is_known() and p.encoding is None:
                # Only try to recognize if the face is big enough (better accuracy)
                if p.region['w'] > 50:
                    found_name, encoding = self._recognize_face(p.region)
                    if encoding is not None:
                        if found_name is not None:
                            p.name = found_name
                            rospy.loginfo(f"ID {fid} identified as {found_name}")
                        p.encoding = encoding
                        current_frame_faces[fid] = p
                    else:
                        rospy.loginfo("No encoding")
            else:
                current_frame_faces[fid] = p

        # 4. SWAP BUFFERS
        # Discard anyone who wasn't seen in this frame (handle "walking away")
        self.faces = current_frame_faces

    # --- RECOGNITION HELPER ---

    def _recognize_face(self, region):
        """
        Crops the face from self.latest_image and compares to DB.
        """
        if self.latest_image is None: 
            return None, None

        # Safe Crop (clamping to image bounds)
        img_h, img_w, _ = self.latest_image.shape
        x = max(0, region['x'])
        y = max(0, region['y'])
        x2 = np.clip(x + region['w'], 0, img_w)
        y2 = np.clip(y + region['h'], 0, img_h)
        
        # Don't crash on empty crop
        if region['w'] < 10 or region['h'] < 10: return None, None #
  
        crop = self.latest_image[y:y2, x:x2]
        
        # Convert BGR (OpenCV) to RGB (face_recognition)
        rgb_crop = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
        
        try:
            # 1. Get Encoding
            unknown_encodings = face_recognition.face_encodings(rgb_crop)
            if not unknown_encodings:
                return None, None # No face found in crop
            
            unknown_encoding = unknown_encodings[0]

            # 2. Compare against DB
            if not self.known_encodings:
                return None, unknown_encoding

            matches = face_recognition.compare_faces(self.known_encodings, unknown_encoding, tolerance=0.5)
            
            if True in matches:
                first_match_index = matches.index(True)
                return self.known_names[first_match_index], unknown_encoding
            
        except Exception as e:
            rospy.logerr(f"Recognition error: {e}")
        
        return None, unknown_encoding

    # --- PUBLIC METHODS ---

    def get_active_faces(self):
        """
        Returns a list of Person objects.
        This is now instant because 'self.faces' is updated in background.
        """
        active_list = []
        now = rospy.Time.now()
        timeout_duration = rospy.Duration(0.5) # 500ms tolerance

        # We iterate over a copy of keys to safely delete if we wanted to
        # But filtering the return list is safer
        
        for fid, person in self.faces.items():
            # Check if the person has a timestamp (new ones do)
            if hasattr(person, 'last_seen'):
                # usage: time_diff = now - last_seen
                age = now - person.last_seen
                
                if age < timeout_duration:
                    active_list.append(person)
                else:
                    # Optional: Clean up self.faces to keep memory low
                    # del self.faces[fid] (careful modifying dict while iterating)
                    pass
            else:
                # If for some reason they have no timestamp, assume stale
                pass

        return active_list

    def register_new_face(self, person):
        """
        Called when the Butler learns a NEW name via voice.
        """

        print(self.faces[person.id].name)
        print(person.name)
        
        self.known_encodings.append(person.encoding)
        self.known_names.append(person.name)
        self.save_faces() # Save to disk
        rospy.loginfo(f"Saved {person.name} to database.")

    # --- DATABASE ---

    def _load_database(self):
        if not os.path.exists(self.database_path):
            return

        try:
            with open(self.database_path, 'r') as f:
                data = json.load(f)
                self.known_names = data['names']
                # Convert list back to numpy array
                self.known_encodings = [np.array(e) for e in data['encodings']]
            rospy.loginfo(f"Loaded {len(self.known_names)} faces from DB.")
        except Exception as e:
            rospy.logerr(f"Failed to load DB: {e}")

    def save_faces(self):
        data = {
            "names": self.known_names,
            # Convert numpy arrays to lists for JSON
            "encodings": [e.tolist() for e in self.known_encodings]
        }
        with open(self.database_path, 'w') as f:
            json.dump(data, f)