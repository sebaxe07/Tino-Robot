import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray, Float32MultiArray
import pyaudio
import numpy as np
import threading
import time
import math
import os
import wave
from ament_index_python.packages import get_package_share_directory

class AudioNode(Node):
    def __init__(self):
        super().__init__('audio_node')
        
        # Publishers - send audio from microphone to robot controller
        self.audio_pub = self.create_publisher(Int16MultiArray, '/audio/mic_input', 10)
        
        # Subscribers - receive audio control from robot controller (from VR)
        # Float32MultiArray[0] = volume (0 to 255), Float32MultiArray[1] = orientation (-1 to 1)
        self.vr_audio_sub = self.create_subscription(
            Float32MultiArray, '/audio/vr_output', self.vr_audio_callback, 10)
        
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
        
        # Audio control parameters
        self.volume = 255.0  # Default max volume (0-255)
        self.orientation = 0.0  # Default centered audio (-1 to 1, where -1 is left, 0 is center, 1 is right)
        
        # Sound generation parameters
        self.is_running = True
        self.sound_thread = None
        self.sound_type = self.declare_parameter('sound_type', 'noface').value  # Options: 'wav_file', 'sine', 'beep', 'complex', 'noface'
        self.frequency_left = 440.0  # A4 note frequency (Hz)
        self.frequency_right = 440.0  # A4 note frequency (Hz)
        self.sample_rate = 44100  # Audio sample rate
        self.sound_chunk = 1024  # Chunk size for sound generation
        self.phase_left = 0.0  # Phase accumulator for left channel
        self.phase_right = 0.0  # Phase accumulator for right channel
        
        # Start the ambient sound thread
        self.sound_thread = threading.Thread(target=self.ambient_sound_thread)
        self.sound_thread.daemon = True
        self.sound_thread.start()
        
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
        """Process incoming audio control parameters from VR via robot controller"""
        try:
            # Check if we have received both volume and orientation values
            if len(msg.data) >= 2:
                # Update volume (0-255)
                self.volume = max(0.0, min(255.0, msg.data[0]))
                
                # Update orientation (-1 to 1)
                self.orientation = max(-1.0, min(1.0, msg.data[1]))
                
                self.get_logger().info(f'Updated audio control: volume={self.volume}, orientation={self.orientation}')
            else:
                self.get_logger().warn('Received incomplete audio control data')
                
        except Exception as e:
            self.get_logger().error(f'Audio control error: {str(e)}')
            
    def ambient_sound_thread(self):
        """Thread function to generate or play audio with real-time volume and stereo control"""
        try:
            self.get_logger().info(f'Starting ambient sound thread with sound type: {self.sound_type}')
            
            # Handle different sound types
            if self.sound_type == 'wav_file':
                # Define the path to the audio file
                # Try to find an appropriate sound file
                wav_file_path = None
                
                # Try the ambientTino.wav file in the assets directory
                potential_path = os.path.join('/home/orinano/Tino-Robot/assets', 'ambientTino.wav')
                if os.path.exists(potential_path):
                    wav_file_path = potential_path
                    self.get_logger().info(f'Found sound file: {wav_file_path}')
                
                # If no suitable file found, create a default sound
                if wav_file_path is None:
                    self.get_logger().warn('No suitable sound file found. Using generated sound.')
                    self.generate_default_sound()
                    wav_file_path = '/tmp/no_face_sound.wav'
            
            # Handle wav file playback
            if self.sound_type == 'wav_file':
                # Open the wav file
                with wave.open(wav_file_path, 'rb') as wf:
                    # Get wav file properties
                    channels = wf.getnchannels()
                    sample_width = wf.getsampwidth()
                    frame_rate = wf.getframerate()
                    n_frames = wf.getnframes()
                    
                    # Read all frames from the wav file
                    self.get_logger().info(f'Loading sound file: {wav_file_path}, {channels} channels, {frame_rate} Hz')
                    all_frames = wf.readframes(n_frames)
                
                # Convert audio data to numpy array for easier manipulation
                if sample_width == 2:  # 16-bit audio
                    audio_data = np.frombuffer(all_frames, dtype=np.int16)
                elif sample_width == 1:  # 8-bit audio
                    audio_data = np.frombuffer(all_frames, dtype=np.uint8)
                    audio_data = (audio_data.astype(np.int16) - 128) * 256
                else:
                    self.get_logger().error(f'Unsupported sample width: {sample_width}')
                    return
                
                # Reshape for stereo if needed
                if channels == 2:
                    audio_data = audio_data.reshape(-1, 2)
                else:
                    # If mono, duplicate to create stereo
                    audio_data = np.column_stack((audio_data, audio_data))
                    channels = 2
                
                # Open an audio output stream for playback
                output_stream = self.p.open(
                    format=self.audio_format,
                    channels=channels,
                    rate=frame_rate,
                    output=True,
                    frames_per_buffer=self.sound_chunk
                )
                
                # Main playback loop for WAV file
                position = 0
                while self.is_running and rclpy.ok():
                    try:
                        # Calculate volume scaling factor (0.0 to 1.0)
                        volume_scale = self.volume / 255.0
                        
                        # Calculate stereo balance based on orientation (-1 to 1)
                        # -1 is full left, 0 is center, 1 is full right
                        left_vol = volume_scale * (1.0 - max(0.0, self.orientation))
                        right_vol = volume_scale * (1.0 + min(0.0, self.orientation))
                        
                        # Get the chunk of audio to play
                        end_pos = min(position + self.sound_chunk, len(audio_data))
                        chunk_data = audio_data[position:end_pos].copy()
                        
                        # Apply volume and stereo balance
                        chunk_data[:, 0] = (chunk_data[:, 0] * left_vol).astype(np.int16)  # Left channel
                        chunk_data[:, 1] = (chunk_data[:, 1] * right_vol).astype(np.int16)  # Right channel
                        
                        # Write the chunk to the output stream
                        output_stream.write(chunk_data.tobytes())
                        
                        # Update position and loop if reached the end
                        position = end_pos
                        if position >= len(audio_data):
                            position = 0  # Loop back to the beginning
                    except Exception as e:
                        self.get_logger().error(f'WAV playback error: {str(e)}')
                        time.sleep(0.1)  # Wait a bit before trying again
            
            # Handle real-time synthesis modes
            else:
                # Open an audio output stream for stereo output
                output_stream = self.p.open(
                    format=self.audio_format,
                    channels=2,  # Always stereo for synthesized sound
                    rate=self.sample_rate,
                    output=True,
                    frames_per_buffer=self.sound_chunk
                )
                
                # Parameters for sine/beep modes
                base_freq = 220.0  # A3 note for pure sine mode
                beep_interval = 1.0  # seconds between beeps
                beep_duration = 0.2  # seconds for each beep
                beep_timer = 0
                beep_active = False
                
                # For complex mode
                complex_lfo_phase = 0.0
                
                # Main sound generation loop
                while self.is_running and rclpy.ok():
                    try:
                        # Calculate volume scaling factor (0.0 to 1.0)
                        volume_scale = self.volume / 255.0
                        
                        # Calculate stereo balance based on orientation (-1 to 1)
                        left_vol = volume_scale * (1.0 - max(0.0, self.orientation))
                        right_vol = volume_scale * (1.0 + min(0.0, self.orientation))
                        
                        # Generate samples based on selected sound type
                        samples = np.zeros((self.sound_chunk, 2), dtype=np.int16)
                        
                        # Simple sine wave generator
                        if self.sound_type == 'sine':
                            for i in range(self.sound_chunk):
                                # Generate sine wave sample
                                sample = math.sin(self.phase_left) * 32767.0
                                
                                # Apply volume to left and right channels
                                samples[i, 0] = int(sample * left_vol)  # Left channel
                                samples[i, 1] = int(sample * right_vol)  # Right channel
                                
                                # Update phase (frequency varies with orientation)
                                freq = base_freq * (1.0 + 0.2 * self.orientation)  # Shift pitch slightly based on orientation
                                self.phase_left += 2 * math.pi * freq / self.sample_rate
                                if self.phase_left > 2 * math.pi:
                                    self.phase_left -= 2 * math.pi
                        
                        # Simple beep generator
                        elif self.sound_type == 'beep':
                            beep_timer += self.sound_chunk / self.sample_rate
                            
                            # Toggle beep state
                            if beep_timer >= beep_interval:
                                beep_timer = 0
                                beep_active = True
                            
                            # Turn off beep after duration
                            if beep_active and beep_timer >= beep_duration:
                                beep_active = False
                            
                            # Generate beep samples
                            if beep_active:
                                for i in range(self.sound_chunk):
                                    # Simple sine wave for beep
                                    sample = math.sin(self.phase_left) * 32767.0
                                    
                                    # Apply volume to left and right channels
                                    samples[i, 0] = int(sample * left_vol)
                                    samples[i, 1] = int(sample * right_vol)
                                    
                                    # Update phase
                                    self.phase_left += 2 * math.pi * base_freq / self.sample_rate
                                    if self.phase_left > 2 * math.pi:
                                        self.phase_left -= 2 * math.pi
                        
                        # Complex modulated sound (similar to No-Face sound but simpler)
                        elif self.sound_type == 'complex':
                            # Update LFO for complex sound
                            complex_lfo_phase += 2 * math.pi * 0.1 / self.sample_rate * self.sound_chunk
                            if complex_lfo_phase > 2 * math.pi:
                                complex_lfo_phase -= 2 * math.pi
                                
                            lfo_mod = math.sin(complex_lfo_phase) * 5.0
                            
                            for i in range(self.sound_chunk):
                                # Main tone with modulation
                                main_tone = math.sin(self.phase_left) * 0.5
                                second_tone = math.sin(self.phase_left * 2.0) * 0.3
                                third_tone = math.sin(self.phase_left * 1.5) * 0.2
                                
                                # Combine tones
                                sample = (main_tone + second_tone + third_tone) * 32767.0
                                
                                # Apply volume to left and right channels
                                samples[i, 0] = int(sample * left_vol)
                                samples[i, 1] = int(sample * right_vol)
                                
                                # Update phase with modulation
                                freq = base_freq + lfo_mod
                                self.phase_left += 2 * math.pi * freq / self.sample_rate
                                if self.phase_left > 2 * math.pi:
                                    self.phase_left -= 2 * math.pi
                        
                        # No-Face eerie sound (complex, ethereal, mysterious)
                        elif self.sound_type == 'noface':
                            # Create more complex modulation parameters for No-Face sound
                            
                            # Update and use multiple LFOs for complex modulation
                            if not hasattr(self, 'noface_params'):
                                # Initialize parameters for No-Face sound if not already done
                                self.noface_params = {
                                    'lfo1_phase': 0.0,        # Slow modulation for base frequency
                                    'lfo2_phase': 0.0,        # Medium-speed modulation for timbre
                                    'lfo3_phase': 0.0,        # Fast modulation for fine detail
                                    'vocal_phase': 0.0,       # For vocal-like elements
                                    'vocal_timer': 0.0,       # Timer for occasional vocal sounds
                                    'vocal_active': False,    # Whether vocal sound is currently active
                                    'intensity': 0.0,         # Dynamic intensity that changes over time
                                    'last_time': time.time(), # For time-based variations
                                    'chord_idx': 0,           # Current position in chord progression
                                    'chord_timer': 0.0        # Timer for chord changes
                                }
                            
                            # Update time-based parameters
                            current_time = time.time()
                            time_delta = current_time - self.noface_params['last_time']
                            self.noface_params['last_time'] = current_time
                            
                            # Update LFO phases
                            self.noface_params['lfo1_phase'] += 2 * math.pi * 0.05 * time_delta  # Very slow LFO (0.05 Hz)
                            self.noface_params['lfo2_phase'] += 2 * math.pi * 0.2 * time_delta   # Slow LFO (0.2 Hz)
                            self.noface_params['lfo3_phase'] += 2 * math.pi * 0.7 * time_delta   # Medium LFO (0.7 Hz)
                            
                            # Wrap phases
                            for key in ['lfo1_phase', 'lfo2_phase', 'lfo3_phase', 'vocal_phase']:
                                if self.noface_params[key] > 2 * math.pi:
                                    self.noface_params[key] -= 2 * math.pi
                            
                            # Update intensity parameter - slowly varies between 0.4 and 1.0
                            self.noface_params['intensity'] = 0.7 + 0.3 * math.sin(self.noface_params['lfo1_phase'])
                            
                            # Update vocal timer and state (vocal-like sounds occur rarely)
                            self.noface_params['vocal_timer'] += time_delta
                            vocal_interval = 8.0 + 4.0 * math.sin(self.noface_params['lfo1_phase'])  # 4-12 seconds between vocalizations
                            
                            # Randomly trigger vocal sounds
                            if not self.noface_params['vocal_active'] and self.noface_params['vocal_timer'] > vocal_interval:
                                if np.random.random() < 0.7:  # 70% chance when timer expires
                                    self.noface_params['vocal_active'] = True
                                    self.noface_params['vocal_duration'] = 1.0 + 1.0 * np.random.random()  # 1-2 seconds
                            
                            # Turn off vocal sound after duration
                            if self.noface_params['vocal_active'] and self.noface_params['vocal_timer'] > vocal_interval + self.noface_params['vocal_duration']:
                                self.noface_params['vocal_active'] = False
                                self.noface_params['vocal_timer'] = 0  # Reset timer
                                self.noface_params['vocal_phase'] = 0  # Reset phase
                            
                            # Update chord progression timer
                            self.noface_params['chord_timer'] += time_delta
                            if self.noface_params['chord_timer'] > 10.0:  # Change chord every 10 seconds
                                self.noface_params['chord_idx'] = (self.noface_params['chord_idx'] + 1) % 4
                                self.noface_params['chord_timer'] = 0
                            
                            # Define chord progression for more interesting harmony
                            # Using frequencies that create an eerie, mysterious feel
                            chord_progressions = [
                                [55.0, 82.5, 110.0, 164.81],  # A1, E2, A2, E3 (minor feeling)
                                [58.27, 87.31, 116.54, 174.61],  # Bb1, F2, Bb2, F3 (shifting up)
                                [55.0, 69.3, 110.0, 138.6],  # A1, C#2, A2, C#3 (diminished feel)
                                [49.0, 73.42, 98.0, 146.83],  # G1, D2, G2, D3 (resolution)
                            ]
                            
                            # Get current chord
                            current_chord = chord_progressions[self.noface_params['chord_idx']]
                            
                            # Calculate modulation values
                            mod1 = math.sin(self.noface_params['lfo1_phase'])  # -1 to 1, very slow
                            mod2 = math.sin(self.noface_params['lfo2_phase'])  # -1 to 1, slow
                            mod3 = math.sin(self.noface_params['lfo3_phase'])  # -1 to 1, medium
                            
                            # Initialize noise filter for smoother noise (if not already done)
                            if not hasattr(self, 'filtered_noise'):
                                self.filtered_noise = 0.0
                            
                            # Generate the audio samples
                            for i in range(self.sound_chunk):
                                # Use multiple frequencies based on current chord for eerie effect
                                # Base tone uses first note of chord - increase the prominence
                                base_freq = current_chord[0] * (1.0 + 0.005 * mod1)  # Reduced detuning (from 0.01)
                                base_tone = math.sin(self.phase_left) * 0.4  # Increased from 0.3
                                
                                # Second tone uses 2nd note of chord
                                second_freq = current_chord[1] * (1.0 + 0.003 * mod2)  # Reduced detuning (from 0.005)
                                second_tone = math.sin(self.phase_right * (second_freq/base_freq)) * 0.25
                                
                                # Third tone uses 3rd note of chord - slightly reduce to clean up the sound
                                third_freq = current_chord[2] * (1.0 + 0.004 * mod3)  # Reduced detuning (from 0.008)
                                third_tone = math.sin((self.phase_left + self.phase_right) * 0.5 * (third_freq/base_freq)) * 0.18  # Reduced from 0.2
                                
                                # Add slight detuning to create dissonance using 4th note - reduce for less noise
                                fourth_freq = current_chord[3] * (1.0 + 0.006 * mod1 * mod2)  # Reduced detuning (from 0.012)
                                fourth_tone = math.sin(self.phase_right * (fourth_freq/base_freq)) * 0.12  # Reduced from 0.15
                                
                                # Add subtle noise with reduced amount based on intensity
                                noise_amount = 0.01 + 0.02 * self.noface_params['intensity']  # Reduced from 0.03+0.04
                                raw_noise = np.random.uniform(-noise_amount, noise_amount)
                                
                                # Apply a simple low-pass filter to the noise (exponential smoothing)
                                alpha = 0.1  # Lower value = smoother noise (0.0-1.0)
                                self.filtered_noise = alpha * raw_noise + (1 - alpha) * self.filtered_noise
                                noise = self.filtered_noise
                                
                                # Add occasional vocal-like sound
                                vocal_sound = 0
                                if self.noface_params['vocal_active']:
                                    # Update vocal phase
                                    vocal_freq = 110.0 * (1.0 + 0.2 * mod2) * (1.5 + mod1)  # Varies between 110-330 Hz
                                    self.noface_params['vocal_phase'] += 2 * math.pi * vocal_freq / self.sample_rate
                                    
                                    # Create a formant-like sound to simulate vocalizations
                                    formant1 = math.sin(self.noface_params['vocal_phase']) * 0.5
                                    formant2 = math.sin(self.noface_params['vocal_phase'] * 2.7) * 0.3
                                    formant3 = math.sin(self.noface_params['vocal_phase'] * 3.5) * 0.2
                                    
                                    # Combine formants with envelope
                                    vocal_progress = (self.noface_params['vocal_timer'] - vocal_interval) / self.noface_params['vocal_duration']
                                    vocal_envelope = math.sin(vocal_progress * math.pi) if 0 <= vocal_progress <= 1 else 0
                                    vocal_sound = (formant1 + formant2 + formant3) * vocal_envelope * 0.7
                                
                                # Add more subtle tremolo effect (amplitude variation)
                                tremolo = 0.92 + 0.08 * math.sin(8.0 * self.noface_params['lfo3_phase'] + i/self.sound_chunk * 2 * math.pi)
                                
                                # Combine all elements with dynamic intensity and tremolo
                                main_sound = (base_tone + second_tone + third_tone + fourth_tone) * tremolo * self.noface_params['intensity']
                                
                                # Final mix with vocal sound and noise
                                sample = (main_sound + vocal_sound + noise) * 32767.0
                                
                                # Apply volume to left and right channels with slight variations for stereo field
                                stereo_spread = 0.1 * math.sin(self.noface_params['lfo2_phase'] + i/self.sound_chunk * 0.5)
                                left_factor = left_vol * (1.0 - stereo_spread)
                                right_factor = right_vol * (1.0 + stereo_spread)
                                
                                samples[i, 0] = int(sample * left_factor)
                                samples[i, 1] = int(sample * right_factor)
                                
                                # Slowly modulate phase increment for main tone variation
                                freq_mod = 1.0 + 0.1 * mod2 + 0.05 * mod3
                                base_freq_with_mod = base_freq * freq_mod
                                
                                # Update phase accumulators for next sample
                                self.phase_left += 2 * math.pi * base_freq_with_mod / self.sample_rate
                                self.phase_right += 2 * math.pi * (base_freq_with_mod * 1.002) / self.sample_rate  # Slightly detuned for right channel
                                
                                # Wrap phases
                                if self.phase_left > 2 * math.pi:
                                    self.phase_left -= 2 * math.pi
                                if self.phase_right > 2 * math.pi:
                                    self.phase_right -= 2 * math.pi
                        
                        # Write samples to output stream
                        output_stream.write(samples.tobytes())
                    
                    except Exception as e:
                        self.get_logger().error(f'Sound playback error: {str(e)}')
                        time.sleep(0.1)  # Wait a bit before trying again
            
            # Clean up
            output_stream.stop_stream()
            output_stream.close()
            
        except Exception as e:
            self.get_logger().error(f'Ambient sound thread error: {str(e)}')
    
    def generate_default_sound(self):
        """Generate a default eerie sound for No-Face and save as WAV file"""
        try:
            # Sound parameters - create an eerie, mysterious sound
            duration = 10.0  # 10 second sound that will loop
            sample_rate = 44100
            n_samples = int(duration * sample_rate)
            
            # Create stereo audio array
            audio_data = np.zeros((n_samples, 2), dtype=np.int16)
            
            # Base frequencies for mysterious sound
            base_freqs = [55.0, 110.0, 164.81, 195.99]  # A1, A2, E3, G3
            
            # Generate a complex, eerie sound
            t = np.linspace(0, duration, n_samples, False)
            
            # Create multiple layers with irregular modulations
            layer1 = 0.4 * np.sin(2 * np.pi * base_freqs[0] * t + 0.2 * np.sin(2 * np.pi * 0.1 * t))
            layer2 = 0.3 * np.sin(2 * np.pi * base_freqs[1] * t * (1 + 0.01 * np.sin(2 * np.pi * 0.17 * t)))
            layer3 = 0.2 * np.sin(2 * np.pi * base_freqs[2] * t) * (0.5 + 0.5 * np.sin(2 * np.pi * 0.13 * t))
            layer4 = 0.1 * np.sin(2 * np.pi * base_freqs[3] * t + 0.3 * np.cos(2 * np.pi * 0.19 * t))
            
            # Add some noise for texture (very subtle)
            noise = 0.05 * np.random.uniform(-1, 1, n_samples)
            
            # Combine all layers
            signal = layer1 + layer2 + layer3 + layer4 + noise
            
            # Apply gentle envelope for smooth looping
            envelope = np.ones(n_samples)
            fade_len = int(0.5 * sample_rate)  # 0.5 sec fade in/out
            envelope[:fade_len] = np.linspace(0, 1, fade_len)
            envelope[-fade_len:] = np.linspace(1, 0, fade_len)
            
            # Apply envelope
            signal = signal * envelope
            
            # Convert to int16, ensuring proper scaling to avoid clipping
            max_val = np.max(np.abs(signal))
            if max_val > 0:
                signal = signal * (32767 / max_val)
            
            # Create stereo channels
            audio_data[:, 0] = signal.astype(np.int16)  # Left
            audio_data[:, 1] = signal.astype(np.int16)  # Right
            
            # Save as WAV file
            with wave.open('/tmp/no_face_sound.wav', 'wb') as wf:
                wf.setnchannels(2)
                wf.setsampwidth(2)  # 2 bytes = 16 bits
                wf.setframerate(sample_rate)
                wf.writeframes(audio_data.tobytes())
                
            self.get_logger().info('Created default sound file at /tmp/no_face_sound.wav')
            
        except Exception as e:
            self.get_logger().error(f'Error generating default sound: {str(e)}')
    
    def cleanup(self):
        """Clean up audio resources"""
        # Stop all audio threads
        self.is_running = False
        if hasattr(self, 'sound_thread') and self.sound_thread and self.sound_thread.is_alive():
            self.sound_thread.join(timeout=1.0)
            
        # Close audio streams    
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