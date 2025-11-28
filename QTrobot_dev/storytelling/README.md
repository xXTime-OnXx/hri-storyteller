## 

In one terminal execute:
```
rosrun audio play_file.py
```

Then in the other terminal run:
```
python3 /root/catkin_ws/src/QTrobot_dev/storytelling/src/temp/voices.py
```

TODO: Find a new voice, which fits the storyteller (male, natural, good to understand)

TODO: rosservice -> rostopic

TODO: push waiting audios to PC and implement logic to integrate them

TODO: implement logic to integrate waiting wavs


## Storytelling Prgoram

```
rosrun storytelling main.py
```

## Telly Monolog

**Intro:**
Hi, I am Telly an interactive Storyteller

**Topic Selection:**
What would you like the story to be about?

Great! Let me tell you a story about {topic}

**Storytelling:**
loop:
    [generated story]

    What should happen next? {topic}

**Outro:**
And that's the end of our story! Thank you for this wonderful adventure together!


## Change Volume
```
rosservice call /qt_robot/setting/setVolume "volume: 80"
```