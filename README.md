# GPT chatbot for PR2 curiosity paper

this chatbot is used ot generate curious questions! the program was run on our PR2 robot, and it plays the part of a curious and helpful robot, in order to study the effects of curiosity in a collaborative task.

this chatbot uses gpt-4o-mini for text generation, vosk for speech-to-text, and azure-cloudservicess-speech for speech synthesis. with all that, it asks the user a curious question about a given object.

## ROS repo dependencies

- [voskros (speech recognition)] (<https://github.com/bob-ros2/voskros>)

## RUN IT

i think thats it, then just

```pip install -r requirements.txt```

and 

```colcon build```

and

```ros2 launch gpt_listener gpt.launch.py```


im open to pull requests lol
