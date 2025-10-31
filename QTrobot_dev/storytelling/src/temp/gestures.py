#!/usr/bin/env python3

import rospy
from qt_gesture_controller.srv import gesture_play
import time

def show_all_emotions():
    # Initialize ROS node
    rospy.wait_for_service('/qt_robot/gesture/play')
    
    # List of all QT robot emotions
    emotions = [
        "QT/monkey"
    ]
    
    # Wait for the emotion service to be available
    rospy.wait_for_service('/qt_robot/gesture/play')
    
    # Create service proxy
    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
    
    print("Starting gesture display demo...")
    
    # Loop through each emotion
    for i, emotion in enumerate(emotions, 1):
        try:
            # Extract just the emotion name for display
            emotion_name = emotion.split('/')[1]
            
            # Print to console
            print(f"[{i}/{len(emotions)}] Displaying gesture: {emotion_name}")
            
            # Show the emotion
            gesturePlay(emotion, 0)
            
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