import rospy
from input_service import InputService

if __name__ == '__main__':
    try:
        rospy.init_node('voice_test_node')
        input_handler = InputService()

        rospy.loginfo("--- Starting simple conversation ---")
        
        input_1 = input_handler.get_speech_input()
        rospy.loginfo(input_1)

    except rospy.ROSInterruptException:
        pass