#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#これはサービスノードで、特定の単語を識別したら認識を終了しresponseを返すノードです


import os
import sys
import json
import queue
import vosk
import sounddevice as sd
from mmap import MAP_SHARED

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from ros2_vosk_msgs.msg import SpeechRecognition
from ros2_vosk_msgs.srv import SpeechRecognitionWord
from std_msgs.msg import String, Bool

from ros2_vosk import vosk_ros_model_downloader as downloader

class vosk_sr(Node):
    def __init__(self):
        super().__init__('vosk')
        self.declare_parameter('vosk/model', "vosk-model-en-us-0.22")
        model_name = self.get_parameter('vosk/model').get_parameter_value().string_value

        package_path = get_package_share_directory('ros2_vosk')
        
        models_dir = os.path.join(package_path, 'models')
        model_path = os.path.join(models_dir, model_name)
        
        if not os.path.exists(model_path):
            print (f"model '{model_name}' not found in '{models_dir}'! Please use the GUI to download it or configure an available model...")
            model_downloader = downloader.model_downloader()
            model_downloader.execute()
            model_name = model_downloader.model_to_download
        
        if not self.has_parameter('vosk/model'):
            new_parameter = rclpy.parameter.Parameter('vosk/model', rclpy.Parameter.Type.STRING, model_name)
            self.set_parameters([new_parameter])

        self.tts_status = False

        # ROS node initialization
        self.pub_vosk = self.create_publisher(SpeechRecognition, 'speech_recognition/vosk_result',10)
        self.pub_final = self.create_publisher(String, 'speech_recognition/final_result', 10)
        self.pub_partial = self.create_publisher(String, 'speech_recognition/partial_result', 10)
        self.service_vosk = self.create_service(SpeechRecognitionWord, 'speech_recognition/word', self.srv_callback)

        self.rate = self.create_rate(100)

        self.msg = SpeechRecognition()

        self.q = queue.Queue()

        self.input_dev_num = sd.query_hostapis()[0]['default_input_device']
        if self.input_dev_num == -1:
            self.get_logger().fatal('No input device found')
            raise ValueError('No input device found, device number == -1')

        device_info = sd.query_devices(self.input_dev_num, 'input')
        # soundfile expects an int, sounddevice provides a float:
        
        self.declare_parameter('vosk/sample_rate', int(device_info['default_samplerate']))
        self.samplerate = int(device_info['default_samplerate'])

        new_parameter = rclpy.parameter.Parameter('vosk/sample_rate', rclpy.Parameter.Type.INTEGER, self.samplerate)
        self.set_parameters([new_parameter])

        self.model = vosk.Model(model_path)

        #TODO GPUInit automatically selects a CUDA device and allows multithreading.
        # gpu = vosk.GpuInit() #TODO

    def srv_callback(self, request, response):
        self.model_name = request.model_name
        custom_vocabulary = request.custom_vocabulary
        try:

            with sd.RawInputStream(samplerate=self.samplerate, blocksize=16000, device=self.input_dev_num, dtype='int16',
                               channels=1, callback=self.stream_callback):
                self.get_logger().debug('Started customed recording')
                
                rec = vosk.KaldiRecognizer(self.model, self.samplerate,json.dumps(custom_vocabulary))
                print("Vosk is ready to listen!!!!!!!!!!!!!!!!!!")
                isRecognized = False
                isRecognized_partially = False

                while True:
                    self.tts_status_listener()

                    if self.tts_status == True:
                        # If the text to speech is operating, clear the queue
                        with self.q.mutex:
                            self.q.queue.clear()
                        rec.Reset()

                    elif self.tts_status == False:
                        data = self.q.get()
                        if rec.AcceptWaveform(data):

                            # In case of final result
                            result = rec.FinalResult()

                            diction = json.loads(result)
                            lentext = len(diction["text"])

                            if lentext > 2:
                                result_text = diction["text"]
                                self.get_logger().info(result_text)
                                isRecognized = True
                            else:
                                isRecognized = False
                            # Resets current results so the recognition can continue from scratch
                            rec.Reset()
                        else:
                            # In case of partial result
                            result_partial = rec.PartialResult()
                            if (len(result_partial) > 20):

                                isRecognized_partially = True
                                partial_dict = json.loads(result_partial)
                                partial = partial_dict["partial"]

                        if (isRecognized is True):

                            self.msg.is_speech_recognized = True
                            self.msg.time_recognized = self.get_clock().now().to_msg()
                            self.msg.final_result = result_text
                            self.msg.partial_result = "unk"
                            self.pub_vosk.publish(self.msg)
                            """
                            #rclpy.sleep(0.1)
                            string_msg = String()
                            string_msg.data = result_text
                            self.pub_final.publish(string_msg)
                            """
                            if result_text in custom_vocabulary:
                                response.is_speech_recognized = True
                                response.final_result = result_text
                                self.get_logger().info(f"Recognized word: {result_text}")
                                return response
                            isRecognized = False


                        elif (isRecognized_partially is True):
                            if partial != "unk":
                                self.msg.is_speech_recognized = False
                                self.msg.time_recognized = self.get_clock().now().to_msg()
                                self.msg.final_result = "unk"
                                self.msg.partial_result = partial
                                self.pub_vosk.publish(self.msg)
                                """
                                #rclpy.sleep(0.1)
                                string_msg = String()
                                string_msg.data = partial
                                self.pub_partial.publish(string_msg)
                                partial = "unk"
                                """
                                isRecognized_partially = False
                                
        except Exception as e:
            exit(type(e).__name__ + ': ' + str(e))
        except KeyboardInterrupt:
            self.get_logger().info("Stopping the VOSK speech recognition node...")
            #rclpy.sleep(1)
            print("node terminated")

    
    def cleanup(self):
        self.get_logger().warn("Shutting down VOSK speech recognition node...")
    
    def stream_callback(self, indata, frames, time, status):
        #"""This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        self.q.put(bytes(indata))
        
    def tts_get_status(self,msg):
        self.tts_status = msg.data

    def tts_status_listener(self):
        self.create_subscription(Bool, '/tts/status', self.tts_get_status, 10)

    
def main(args=None):
    rclpy.init(args=args)
    rec = vosk_sr()
    try:
        rclpy.spin()
    except KeyboardInterrupt as e:
        rec.get_logger().fatal("Error occurred! Stopping the vosk speech recognition node...")
        #rclpy.sleep(1)
        print("node terminated")
    rec.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()