#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from qt_nuitrack_app.msg import Faces
from cv_bridge import CvBridge

class OneShotAnnotated:
    def __init__(self):
        rospy.init_node("one_shot_annotated")
        self.bridge = CvBridge()

        self.frame = None
        self.faces = None
        self.saved = False

        rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)
        rospy.Subscriber("/qt_nuitrack_app/faces", Faces, self.face_cb)

        rospy.loginfo("Waiting for one camera frame & Nuitrack data...")
        self.main_loop()

    def image_cb(self, msg):
        if self.frame is None:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def face_cb(self, msg):
        if self.faces is None:
            self.faces = msg.faces

    def main_loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.saved:
            if self.frame is not None and self.faces is not None:
                self.save_annotated()
                self.saved = True
            rate.sleep()

    def save_annotated(self):
        img = self.frame.copy()
        h, w, _ = img.shape

        for f in self.faces:
            # Convert normalized Nuitrack box → pixels
            x = int(f.rectangle[0] * w)
            y = int(f.rectangle[1] * h)
            ww = int(f.rectangle[2] * w)
            hh = int(f.rectangle[3] * h)

            # Draw bounding box
            cv2.rectangle(img, (x, y), (x + ww, y + hh), (0, 255, 0), 2)

            # Gender
            gender = "male" if f.gender == "male" else "female"

            # Emotion classification
            emotions = {
                "angry":     f.emotion_angry,
                "happy":     f.emotion_happy,
                "surprised": f.emotion_surprise
            }
            dominant_emotion = max(emotions, key=emotions.get)
            dom_score = emotions[dominant_emotion]

            # Build label text
            label_top = f"ID {f.id} | {gender}"
            label_bottom = f"{dominant_emotion} ({dom_score:.2f})"

            # Draw text
            cv2.putText(img, label_top, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            cv2.putText(img, label_bottom, (x, y + hh + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            # Optional: print raw values for debugging
            #cv2.putText(img, f"A:{f.emotion_angry:.2f} H:{f.emotion_happy:.2f} S:{f.emotion_surprise:.2f}",
            #             (x, y + hh + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        filename = "annotated_snapshot.jpg"
        cv2.imwrite(filename, img)
        rospy.loginfo(f"Saved annotated image to {filename}")


if __name__ == "__main__":
    OneShotAnnotated()
