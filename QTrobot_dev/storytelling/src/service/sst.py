#!/usr/bin/env python3

import rospy
from input_service import InputService
from face_tracker_service import FaceTrackerService

if __name__ == '__main__':
    try:
        rospy.init_node('service_test_node')
        #input_handler = InputService()
        face_tracker = FaceTrackerService()

        face_tracker.start_face_tracking()

        rospy.spin()

        #rospy.loginfo("--- Starting simple conversation ---")
        
        #input_1 = input_handler.get_speech_input()

    except rospy.ROSInterruptException as e:
        rospy.loginfo(e)
        pass