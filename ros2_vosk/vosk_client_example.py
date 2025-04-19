import sys 

from ros2_vosk_msgs.srv import SpeechRecognitionWord
import rclpy
from rclpy.node import Node

class Vosk_client(Node):
    def __init__(self):
        super().__init__('vosk_client')
        self.cli = self.create_client(SpeechRecognitionWord, 'speech_recognition/word')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpeechRecognitionWord.Request()

    def send_request(self):
        self.req.model_name = "vosk-model-en-us-0.22"
        self.req.custom_vocaburary = ["coke", "green tea", "orange juice", "soda", "wine","water"]
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=100.0)
        if self.future.done():
            if self.future.result() is not None:
                if self.future.result().is_speech_recognized:
                    self.get_logger().info('Request succeeded')
                    return self.future.result().final_result
                else:
                    self.get_logger().info('Request failed')
                    return None
    

def main(args=None):
    rclpy.init(args=args)

    vosk_client = Vosk_client()
    vosk_client.send_request()
    rclpy.spin(vosk_client)

    vosk_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()