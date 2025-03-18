# GPT chatbot for PR2 curiosity paper

this chatbot uses gpt for text generation, vosk for speech recogntion, and azure-cloudservices for speech. with that it asks the user a curious question about a given object

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
