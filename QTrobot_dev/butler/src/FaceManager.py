import rospy
import json
from custom_interfaces.srv import FaceDetection
from Person import Person

class FaceManager:

    def __init__(self):
        self.faces = {}

        rospy.wait_for_service("/custom/cv/deep_face/detect")
        self.detect = rospy.ServiceProxy("/custom/cv/deep_face/detect", FaceDetection)

    def get_active_faces(self):
        active_faces = []

        resp = self.detect()
        face_data = json.loads(resp.json_result)

        if face_data is None:
            return active_faces

        for face_id, attributes in face_data.items():
            if attributes is None or attributes[0]["face_confidence"] < 0.8:
                continue

            attributes = attributes[0]

            if face_id not in self.faces:
                self.faces[face_id] = Person(
                    id=face_id,
                    gender=attributes["dominant_gender"],
                    race=attributes["dominant_race"],
                    emotion=attributes["dominant_emotion"],
                    region=attributes["region"]
                )

            # update dynamic fields
            p = self.faces[face_id]
            p.emotion = attributes["dominant_emotion"]
            p.region = attributes["region"]

            active_faces.append(p)

        return active_faces

    def set_name(self, id, name):
        person = self.faces[id]
        person.name = name

    def save_faces(self):
        pass
