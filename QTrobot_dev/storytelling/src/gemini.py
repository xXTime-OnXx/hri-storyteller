#!/usr/bin/env python3
import os
import requests
import rospy
import json
from std_msgs.msg import String

class GeminiAdapter:
    def __init__(self, model_name="gemini-2.5-flash:generateContent"):
        self.api_key = os.environ.get("GENAI_API_KEY")
        if not self.api_key:
            raise Exception("GENAI_API_KEY environment variable not set")
        
        self.model_name = model_name
        self.url = f"https://generativelanguage.googleapis.com/v1beta/models/{self.model_name}"
        rospy.loginfo(f"GENAI_API_KEY loaded from environment")
        rospy.loginfo(f"Using Gemini model: {self.model_name}")

    def build_payload(self, text):
        return {
            "contents": [
                {
                    "parts": [
                        {"text": text}
                    ]
                }
            ]
        }

    def request(self, text):
        headers = {
            "x-goog-api-key": self.api_key,
            "Content-Type": "application/json"
        }
        payload = self.build_payload(text)
        rospy.loginfo(f"Gemini request body: {json.dumps(payload)}")

        try:
            res = requests.post(self.url, headers=headers, json=payload)
        except requests.RequestException as e:
            rospy.logerr(f"Request failed: {e}")
            raise

        if res.status_code != 200:
            rospy.logerr(f"Request to Gemini failed: {res.status_code} {res.text}")
            raise Exception(f"Request to Gemini failed: {res.status_code} {res.text}")

        data = res.json()
        try:
            return data['candidates'][0]['content']['parts'][0]['text']
        except (KeyError, IndexError):
            rospy.logerr(f"Unexpected response structure: {json.dumps(data)}")
            raise

def talk_text(pub, text):
    text = text[:200]
    msg = String()
    msg.data = text
    pub.publish(msg)
    rospy.loginfo(f"Published to /qt_robot/behavior/talkText: {text}")


if __name__ == "__main__":
    rospy.init_node("gemini_adapter_pub")
    
    # Publisher for QTrobot speech
    talk_pub = rospy.Publisher("/qt_robot/behavior/talkText", String, queue_size=10)
    rospy.sleep(1)  # Give ROS time to register the publisher
    
    adapter = GeminiAdapter()
    prompt = "Hello! Let's start a story about QTrobot. QTrobot discovers a new emotion for the first time."
    
    try:
        story = adapter.request(prompt)
        rospy.loginfo(f"Generated text:\n{story}")
        talk_text(talk_pub, story)
    except Exception as e:
        rospy.logerr(f"Error during Gemini request: {e}")