import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import numpy as np
import pyaudio

class AudioLoopback(Node):
    def __init__(self):
        super().__init__('audio_loopback')
        
        # PyAudio setup
        self.p = pyaudio.PyAudio()
        self.output_stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            output=True
        )
        
        # Subscribe to microphone audio
        self.mic_sub = self.create_subscription(
            Int16MultiArray, 
            '/vr_in/audio_output', 
            self.audio_callback, 
            10)
        
        self.get_logger().info('Audio loopback started - you should hear your microphone')
    
    def audio_callback(self, msg):
        """Play incoming audio immediately"""
        try:
            # Convert int16 array back to bytes
            audio_bytes = np.array(msg.data, dtype=np.int16).tobytes()
            self.output_stream.write(audio_bytes)
        except Exception as e:
            self.get_logger().error(f'Audio playback error: {str(e)}')
    
    def cleanup(self):
        """Clean up audio resources"""
        if hasattr(self, 'output_stream') and self.output_stream:
            self.output_stream.stop_stream()
            self.output_stream.close()
        if self.p:
            self.p.terminate()

def main(args=None):
    rclpy.init(args=args)
    loopback = AudioLoopback()
    
    try:
        print("Audio loopback running. Press Ctrl+C to exit...")
        rclpy.spin(loopback)
    except KeyboardInterrupt:
        print("Audio loopback shutting down...")
    finally:
        loopback.cleanup()
        loopback.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()