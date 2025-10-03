#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
    rospy.init_node('my_python_node')
    rospy.loginfo("my_python_node started!")

    param1 = rospy.get_param('~param1', 'default_value')
    rospy.loginfo("value of param1 is %s", param1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")