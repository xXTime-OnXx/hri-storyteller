import json
from std_msgs.msg import String
from qt_robot_interface.srv import emotion_show, gesture_show

def talk_pub(string):
    rospy.Publisher("/qt_robot/behavior/talkText", string)

def show_emotion(emotion):
    rospy.ServiceProxy('/qt_robot/emotion/show', emotion)

def show_gesture(gesture):
    rospy.ServiceProxy('/qt_robot/gesture/show', gesture)

def publish_story(json_string: str):
    data = json.loads(json_string)

    for chunk in data.get("story_chunks", []):
        text    = chunk.get("text", "")
        emotion = chunk.get("emotion", "")
        motion  = chunk.get("motion", "")

        # --- TALK ---
        talk_pub(string)

        # --- EMOTION ---
        if emotion:
            try:
                show_emotion(emotion)
            except Exception as e:
                rospy.logwarn(f"Emotion call failed: {e}")

        # --- GESTURE ---
        if gesture:
            try:
                show_gesture(gesture)
            except Exception as e:
                rospy.logwarn(f"Gesture call failed: {e}")

        rospy.sleep(0.2)  # small pacing delay if needed


try:
    talk_text(talk_pub, "I'm generating your story now.")
    story_json = adapter.request(prompt)  # Gemini returns valid JSON string
    rospy.loginfo(f"Received JSON:\n{story_json}")

    publish_story(story_json)

except Exception as e:
    rospy.logerr(f"Error during Gemini request or execution: {e}")
