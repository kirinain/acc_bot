
import os
import rclpy
from rclpy.node import Node
import wave
import json
from vosk import Model, KaldiRecognizer
from std_msgs.msg import String

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text')
        self.publisher_ = self.create_publisher(String, 'voice_commands', 10)

        # Load Vosk model
        model_path = "/home/kiti/acc_bot/src/voice_control/model/model"
        model = Model(model_path)
        self.recognizer = KaldiRecognizer(model, 16000)

        # Process recorded audio file
        audio_path = "/home/kiti/acc_bot/src/voice_control/voice_control/audio/ok.wav"  # Update with the correct path
        self.process_audio(audio_path)

    def process_audio(self, file_path):
        """Reads a recorded audio file and converts it to text"""
        if not os.path.exists(file_path):
            self.get_logger().error(f"File not found: {file_path}")
            return

        self.get_logger().info(f"Processing file: {file_path}")

        with wave.open(file_path, "rb") as wf:
            self.get_logger().info(f"Audio properties: {wf.getnchannels()} channels, {wf.getframerate()} Hz")

            while True:
                data = wf.readframes(4000)
                if len(data) == 0:
                    break

                if self.recognizer.AcceptWaveform(data):
                    result = json.loads(self.recognizer.Result())
                    command_text = result.get("text", "")
                    self.get_logger().info(f"Recognized: {command_text}")

                    if command_text:
                        msg = String()
                        msg.data = command_text
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Published: {msg.data}")
                    else:
                        self.get_logger().warn("No speech recognized in this file.")



def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
