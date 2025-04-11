import rclpy
from rclpy.node import Node
from autopatol_interfaces.srv import SpeechText
import espeakng

class Speaker(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.speech_service = self.create_service(SpeechText, 'speech_text', self.speech_text_callback)
        self.speaker_ = espeakng.Speaker()
        self.speaker_.voice = 'zh'
    def speech_text_callback(self, request, response):
        self.get_logger().info('正在准备朗读{request.text}')
        self.speaker_.say(request.text)
        self.speaker_.wait()
        response.result = True
        return response
    
def main():
    rclpy.init()
    node = Speaker('speaker')
    rclpy.spin(node)
    rclpy.shutdown()

       

    