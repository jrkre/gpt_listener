import azure.cognitiveservices.speech as speechsdk
import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class Talker(Node):
    
    def __init__(self):
        super().__init__('talker')
        self.subscription = self.create_subscription(
            String,
            '/speech/tts',
            self.speak_callback,
            10
        )
        
        self.enable_pub = self.create_publisher(String, '/speech/listen_enable', 10)
        
        self.speech_config = speechsdk.SpeechConfig(subscription=os.environ.get('SPEECH_KEY'), region=os.environ.get('SPEECH_REGION'))
        self.audio_config = speechsdk.audio.AudioOutputConfig(use_default_speaker=True)

        self.speech_config.speech_synthesis_voice_name = 'en-US-AvaMultilingualNeural'

        # Initialize the speech synthesizer
        self.speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=self.speech_config)

    def speak_callback(self, msg):
        text_to_speak = msg.data
        self.get_logger().info(f'Talker Speaking: "{text_to_speak}"')
        result = self.speech_synthesizer.speak_text_async(text_to_speak).get()
        
        
        if result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:
            self.get_logger().info('Speech synthesis completed.')
            time.sleep(2)
            
            self.enable_pub.publish(String(data="enable"))
        elif result.reason == speechsdk.ResultReason.Canceled:
            cancellation_details = result.cancellation_details
            self.get_logger().error(f'Speech synthesis canceled: {cancellation_details.reason}')
            if cancellation_details.reason == speechsdk.CancellationReason.Error:
                self.get_logger().error(f'Error details: {cancellation_details.error_details}')
                
                
def main():
    rclpy.init()
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    
    main()