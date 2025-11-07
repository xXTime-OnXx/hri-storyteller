#!/usr/bin/env python3

import rospy
from  std_msgs.msg import String
from audio.srv import PlayAudioFile
from qt_robot_interface.srv import audio_play
from file_transfer import FileTransfer




class PlayAudioFileServer():

    def __init__(self):
        rospy.init_node('play_audio_file_server', anonymous=True)
        service = rospy.Service('qt_robot/audio/play_local_file', PlayAudioFile, self.play_callback)
        
        self.robot_ip = '192.168.100.1'
        self.robot_psk = 'qtrobot'
        self.robot_username  = 'developer'
        try:
            self.file_transfer = FileTransfer(remote_host=self.robot_ip, remote_username=self.robot_username, remote_password=self.robot_psk, remote_path='/home/developer/audio.wav')
        except:
            rospy.logerr("file transfer failed to set up connection with robot")
        
        rospy.wait_for_service('/qt_robot/audio/play')
        try:
            self.qtservice = rospy.ServiceProxy('/qt_robot/audio/play', audio_play)
        except:
            rospy.logerr("could not connect to /qt_robot/audio/play service")
        
        rospy.spin()

    def play_callback(self, req):
        rospy.loginfo("received request to play file: " + req.path)
        path = req.path
        try:
            self.file_transfer.transfer_file(local_path=path)
        except:
            rospy.logerr("failed to transfer audio file")
            return False
        
        try:
            self.qtservice('audio.wav', '/home/developer')
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
        return True


if __name__ == "__main__":
    PlayAudioFileServer()