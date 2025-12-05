import rospy
 
class Person:
 
    def __init__(self, id, gender, race, emotion, region):
        self.id = id
        self.gender = gender
        self.race = race
        self.emotion = emotion
        self.region = region
        self.last_ineraction = None #Gets set later
        self.name = None #Gets set later
        self.sexual_orientation = None
   
    def timeout_active(self):
        if self.last_ineraction is None:
            return False
 
        current = rospy.get_time()
 
        return current - self.last_ineraction < 60 # 60 second timeout
 
    def is_known(self):
        return not self.name is None
 