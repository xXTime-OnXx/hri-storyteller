cmake_minimum_required(VERSION 3.0.2)
project(custom_interfaces)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  sensor_msgs # Sie haben dies bereits, was gut ist.
)

# Füge hier deine Nachrichten-Dateien hinzu
add_message_files(
  FILES
  BoundingBox.msg # Füge deine BoundingBox.msg Nachricht hinzu
)

# Füge hier deine Service-Dateien hinzu
add_service_files(
  DIRECTORY srv
  FILES
  Detectron.srv
  MicrophoneBasedSpeechRecognition.srv
  TopicBasedSpeechRecognition.srv
  FaceDetection.srv
)

# Generiere die Nachrichten und Service-Nachrichten
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs  # Stelle sicher, dass dies hier steht, falls deine Nachrichten sensor_msgs verwenden
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs sensor_msgs # Füge message_runtime hinzu
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
