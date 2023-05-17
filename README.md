ros2_vosk
======================

A ROS2 package for speech-to-text services based on [Vosk](https://github.com/alphacep/vosk-api)

Modified from the ROS1 package [ros_vosk](https://github.com/alphacep/ros-vosk) and tested on ROS2 Galactic
The custom message definitions have been moved to the [ros2_vosk_msgs](https://github.com/redchillipadi/ros2_vosk_msgs) package.

## Tutorials

1. Install Dependencies

  ```bash
  sudo apt install python3-pip
  pip install vosk
  pip install sounddevice
  sudo apt-get install libportaudio2
  ```

  And if you want to use the TTS engine please run:
  ```bash
  sudo apt install espeak
  pip install pyttsx3
  ```  

2. Install this package and the messages to your ros2 workspace

  ```bash
  mkdir -p ~/ros2_ws/src 
  cd ~/ros2_ws/src
  git clone https://github.com/redchillipadi/ros2_vosk
  git clone https://github.com/redchillipadi/ros2_vosk_msgs
  cd ..
  colcon build
  ```
  
3. Launch the node

  Launch the speech recognition node

  ```bash
  ros2 run ros_vosk vosk_node
  ```

## Interface

### Publishing Topics
* speech_recognition/vosk_result    -> vosk_node.py publishes a custom "speech_recognition" message
* speech_recognition/final_result   -> vosk_node.py publishes a simple string with the final result
* speech_recognition/partial_result -> vosk_node.py publishes a simple string with the partial result
* tts/status -> tts_engine.py publishes the state of the engine. True if it is speaking False if it is not. If the status is true vosk_node won't process the audio stream so it won't listen to itself 
* tts/phrase -> tts_engine.py subscribes to this topic in order to speak the given string. Name your desire and it shall be heard by all in the room..

## Author
Angelo Antikatzidis <an.antikatzidis@gmail.com>
Nickolay V. Shmyrev <nshmyrev@gmail.com>
Adrian Grigo <agrigo2001@yahoo.com.au>
