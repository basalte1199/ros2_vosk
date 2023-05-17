#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pyttsx3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool

class tts_engine(Node):

    def __init__(self):
        super().__init__('tts_engine')
        self.create_subscription(String, "tts/phrase", self.callback, 10)
        self.pubStatus = self.create_publisher(Bool, 'tts/status', 10)

    def callback(self, msg):
        phrase = msg.data
        self.say(phrase)
    
    def publish_status(self, isSpeaking):
        # Make the status true if it is speaking and false if it is not
        if isSpeaking == True:
            #rclpy.sleep(0.1)
            self.pubStatus.publish(True)
            self.get_logger().debug("Started Speaking!")
        else:
            #rclpy.sleep(0.1)
            self.pubStatus.publish(False)
            self.get_logger().debug("Finished Speaking..")
    
    def say(self, phrase):

        self.get_logger().info("The robot says: " + phrase)
        self.engine = pyttsx3.init()
        self.engine.connect('started-utterance', self.tts_onStart)
        self.engine.connect('finished-utterance', self.tts_onEnd)
        self.engine.say(phrase,"tts_engine")
        self.engine.runAndWait()

    def tts_onStart(self, name):
        self.get_logger().debug('starting speaking ', name)
        self.publish_status(True)

    def tts_onWord(self, name, location, length):
        self.get_logger().debug('word', name, location, length)

    def tts_onEnd(self, name, completed):
        self.get_logger().debug('finishing speaking', name, completed)
        self.publish_status(False)
        self.engine.endLoop()
   
def main(args=None):
    try:
        rclpy.init(args=args)
        tts = tts_engine()
        rclpy.spin(tts) 
    except KeyboardInterrupt:
        tts.get_logger().info("Stopping tts engine...")
        #rclpy.sleep(1)
        print("node terminated")
    tts.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()