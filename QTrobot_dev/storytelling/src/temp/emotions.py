#!/usr/bin/env python3

import rospy
from qt_robot_interface.srv import emotion_show
import time

def show_all_emotions():
    # Initialize ROS node
    rospy.init_node('emotion_display_demo')
    
    # List of all QT robot emotions
    emotions = [
        "QT/afraid",
        "QT/angry",
        "QT/blowing_raspberry",
        "QT/breathing_exercise",
        "QT/brushing_teeth",
        "QT/brushing_teeth_foam",
        "QT/calmig_down_exercise_nose",
        "QT/calming_down",
        "QT/confused",
        "QT/cry",
        "QT/dirty_face",
        "QT/dirty_face_sad",
        "QT/dirty_face_wash",
        "QT/disgusted",
        "QT/happy",
        "QT/happy_blinking",
        "QT/kiss",
        "QT/neutral",
        "QT/neutral_state_blinking",
        "QT/puffing_the_chredo_eeks",
        "QT/sad",
        "QT/scream",
        "QT/showing_smile",
        "QT/shy",
        "QT/talking",
        "QT/with_a_cold",
        "QT/with_a_cold_cleaning_nose",
        "QT/with_a_cold_sneezing",
        "QT/yawn"
    ]
    
    # Wait for the emotion service to be available
    rospy.wait_for_service('/qt_robot/emotion/show')
    
    # Create service proxy
    emotion_show_service = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)
    
    print("Starting emotion display demo...")
    print(f"Total emotions to display: {len(emotions)}\n")
    
    # Loop through each emotion
    for i, emotion in enumerate(emotions, 1):
        try:
            # Extract just the emotion name for display
            emotion_name = emotion.split('/')[1]
            
            # Print to console
            print(f"[{i}/{len(emotions)}] Displaying emotion: {emotion_name}")
            
            # Show the emotion
            emotion_show_service(emotion)
            
            # Wait for 3 seconds before showing next emotion
            time.sleep(3)
            
        except rospy.ServiceException as e:
            print(f"Service call failed for {emotion}: {e}")
        except Exception as e:
            print(f"Error displaying {emotion}: {e}")
    
    print("\nEmotion display demo completed!")

if __name__ == '__main__':
    try:
        show_all_emotions()
    except rospy.ROSInterruptException:
        print("Demo interrupted by user")
    except KeyboardInterrupt:
        print("\nDemo stopped by user")