import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
import pyaudio
import numpy as np
import threading
import time

class AudioNode(Node):
    def __init__(self):
        super().__init__('audio_node')
        
        # Publishers - send audio from microphone to robot controller
        self.audio_pub = self.create_publisher(Int16MultiArray, '/audio/mic_input', 10)
        
        # Subscribers - receive audio from robot controller (from VR)
        self.vr_audio_sub = self.create_subscription(
            Int16MultiArray, '/audio/vr_output', self.vr_audio_callback, 10)
        
        # PyAudio setup for microphone
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000  # Sample rate
        self.chunk = 1024  # Buffer size
        
        # Start the microphone thread
        self.mic_thread = threading.Thread(target=self.mic_thread_function)
        self.mic_thread.daemon = True
        self.mic_thread.start()
        
        # Initialize output stream as None
        self.output_stream = None
        
        self.get_logger().info('Audio node initialized')
    
    def mic_thread_function(self):
        """Thread function to continuously read from the microphone"""
        try:
            self.stream = self.p.open(
                format=self.audio_format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                frames_per_buffer=self.chunk
            )
            
            while rclpy.ok():
                # Read audio from microphone
                data = self.stream.read(self.chunk, exception_on_overflow=False)
                
                # Convert bytes to int16 array
                audio_array = np.frombuffer(data, dtype=np.int16).tolist()
                
                # Create and publish audio message
                msg = Int16MultiArray()
                msg.data = audio_array
                self.audio_pub.publish(msg)
                
                # Small sleep to prevent high CPU usage
                time.sleep(0.01)
                
        except Exception as e:
            self.get_logger().error(f'Microphone error: {str(e)}')
        finally:
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
            
    def vr_audio_callback(self, msg):
        """Process incoming audio from VR via robot controller"""
        try:
            if not self.output_stream:
                self.output_stream = self.p.open(
                    format=self.audio_format,
                    channels=self.channels,
                    rate=self.rate,
                    output=True
                )
            
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
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.p:
            self.p.terminate()

def main(args=None):
    rclpy.init(args=args)
    audio_node = AudioNode()
    
    try:
        print("Audio node running. Press Ctrl+C to exit...")
        rclpy.spin(audio_node)
    except KeyboardInterrupt:
        print("Audio node shutting down...")
    finally:
        audio_node.cleanup()
        audio_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()