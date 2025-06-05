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
import random
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
                        
                        # Enhanced stereo panning with boosted dominant channel 
                        # Apply non-linear curve to make panning more pronounced
                        orientation_effect = math.pow(abs(self.orientation), 0.7) * (1.0 if self.orientation >= 0 else -1.0)
                        
                        # Calculate stereo balance with enhanced panning effect
                        # When panning to one side, we'll reduce the opposite channel more aggressively
                        # while slightly boosting the dominant channel
                        if orientation_effect >= 0:  # Right side dominant
                            left_vol = volume_scale * (1.0 - (orientation_effect * 1.2))  # Reduce left more aggressively
                            right_vol = volume_scale * (1.0 + (orientation_effect * 0.3))  # Slightly boost right
                        else:  # Left side dominant
                            left_vol = volume_scale * (1.0 + (abs(orientation_effect) * 0.3))  # Slightly boost left
                            right_vol = volume_scale * (1.0 - (abs(orientation_effect) * 1.2))  # Reduce right more aggressively
                            
                        # Ensure we don't exceed safe volume limits
                        left_vol = max(0.0, min(1.0, left_vol))
                        right_vol = max(0.0, min(1.0, right_vol))
                        
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
                        
                        # Enhanced stereo panning with boosted dominant channel
                        # Apply non-linear curve to make panning more pronounced
                        orientation_effect = math.pow(abs(self.orientation), 0.65) * (1.0 if self.orientation >= 0 else -1.0)
                        
                        # Calculate panning intensity for exponential curve (0.0 to 1.0)
                        pan_intensity = min(1.0, abs(self.orientation) * 1.5)
                        
                        # Calculate stereo balance with enhanced panning effect using exponential curve
                        # When panning to one side, we'll reduce the opposite channel exponentially
                        # while slightly boosting the dominant channel
                        if orientation_effect >= 0:  # Right side dominant
                            # Stronger reduction using quadratic falloff for better separation
                            left_vol = volume_scale * (1.0 - (orientation_effect * 1.1 + pan_intensity * pan_intensity * 0.5))
                            right_vol = volume_scale * (1.0 + (orientation_effect * 0.3))  # Slightly boost right
                        else:  # Left side dominant
                            right_vol = volume_scale * (1.0 - (abs(orientation_effect) * 1.1 + pan_intensity * pan_intensity * 0.5))
                            left_vol = volume_scale * (1.0 + (abs(orientation_effect) * 0.3))  # Slightly boost left
                            
                        # Ensure we don't exceed safe volume limits
                        left_vol = max(0.0, min(1.0, left_vol))
                        right_vol = max(0.0, min(1.0, right_vol))
                        
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

                        elif self.sound_type == 'noface':
                            if not hasattr(self, 'noface_params'):
                                self.noface_params = {
                                    'breath_phase': 0.0,
                                    'breath_cycle': 0,
                                    'breath_shape': 0,
                                    'breath_filter': 0.0,
                                    'breath_noise': 0.0,
                                    'last_time': time.time(),
                                    'breath_state': 'inhale',  # 'inhale' or 'exhale'
                                    'breath_duration': 3.0,     # Base duration in seconds
                                    'breath_volume': 0.65       # Slightly reduced from 0.7 to prevent clipping
                                }
                                # Pre-calculated values
                                self.breath_waveform = np.array([math.sin(x * math.pi / 180) for x in range(0, 181)])
                                self.filter_coeff = 0.92
                                # Pre-calculated tone gains to avoid runtime multiplication
                                self.tone_gains = [0.65, 0.45, 0.25]  # Reduced slightly from initial boost

                            params = self.noface_params
                            current_time = time.time()
                            dt = current_time - params['last_time']
                            params['last_time'] = current_time

                            # Calculate volume scaling factor (0.0 to 1.0)
                            volume_scale = self.volume / 255.0
                            
                            # Dramatically enhanced stereo panning with extreme channel separation
                            # Use a more aggressive non-linear curve specifically for noface audio
                            orientation_effect = math.pow(abs(self.orientation), 0.5) * (1.0 if self.orientation >= 0 else -1.0)
                            
                            # Apply extreme panning effect with dramatic channel attenuation
                            if orientation_effect >= 0:  # Right side dominant
                                # Strong left channel reduction based on orientation
                                left_vol = volume_scale * (1.0 - (min(0.95, orientation_effect * 2.0)))
                                # Enhanced right channel boost
                                right_vol = volume_scale * (1.0 + (orientation_effect * 0.5))
                            else:  # Left side dominant
                                # Enhanced left channel boost
                                left_vol = volume_scale * (1.0 + (abs(orientation_effect) * 0.5))
                                # Strong right channel reduction based on orientation
                                right_vol = volume_scale * (1.0 - (min(0.95, abs(orientation_effect) * 2.0)))
                                
                            # Ensure we don't exceed safe volume limits
                            left_vol = max(0.05, min(1.0, left_vol))  # Keep minimal volume to prevent complete silence
                            right_vol = max(0.05, min(1.0, right_vol))

                            # Breathing state machine (unchanged)
                            breath_progress = params['breath_phase'] / (2 * math.pi)
                            
                            if params['breath_state'] == 'inhale' and breath_progress > 0.5:
                                params['breath_state'] = 'exhale'
                                params['breath_duration'] = 2.5 + random.random() * 1.5
                            elif params['breath_state'] == 'exhale' and breath_progress > 1.0:
                                params['breath_state'] = 'inhale'
                                params['breath_volume'] = 0.55 + random.random() * 0.2  # Slightly reduced variation
                                params['breath_phase'] = 0.0
                                params['breath_cycle'] += 1

                            # Update breath phase
                            breath_speed = (2 * math.pi) / (params['breath_duration'] * self.sample_rate / self.sound_chunk)
                            params['breath_phase'] += breath_speed

                            # Generate breath shape
                            breath_pos = min(180, int((params['breath_phase'] % (2 * math.pi)) / (2 * math.pi) * 180))
                            breath_shape = self.breath_waveform[breath_pos]

                            # Base tone components
                            tone1_phase = (self.phase_left * 110 / 60) % (2 * math.pi)
                            tone2_phase = (self.phase_right * 146.83 / 60.1) % (2 * math.pi)
                            tone3_phase = (self.phase_left * 73.42 / 60) % (2 * math.pi)
                            
                            # Generate samples with clipping protection
                            for i in range(self.sound_chunk):
                                # Base tones with pre-calculated gains
                                base_tone = (self.tone_gains[0] * math.sin(tone1_phase) + 
                                            self.tone_gains[1] * math.sin(tone2_phase) +
                                            self.tone_gains[2] * math.sin(tone3_phase))
                                
                                # Filtered breath noise (slightly reduced)
                                raw_noise = random.uniform(-1, 1)
                                params['breath_noise'] = (self.filter_coeff * params['breath_noise'] + 
                                                        (1 - self.filter_coeff) * raw_noise)
                                noise = params['breath_noise'] * (0.07 + 0.1 * breath_shape)  # Further reduced
                                
                                # Apply breath volume envelope with soft limit
                                volume_envelope = min(1.0, params['breath_volume'] * (0.75 + 0.25 * breath_shape))
                                
                                # Nearly eliminate internal panning to let global panning dominate completely
                                internal_pan = 0.03 * math.sin(params['breath_phase'] * 0.5)  # Greatly reduced internal panning
                                
                                # Calculate dramatically enhanced gain for each channel
                                # Base gains from volume envelope with enhanced stereo separation
                                base_gain = volume_envelope * 0.90  # Further reduction to prevent clipping with boosted channels
                                
                                # Apply minimal breathing modulation and maximized global volume/orientation
                                # Start with basic gains
                                final_left_gain = base_gain * (1.0 - internal_pan * 0.05) * left_vol  # Minimal internal pan effect
                                final_right_gain = base_gain * (1.0 + internal_pan * 0.05) * right_vol
                                
                                # Calculate extreme orientation intensity with higher scaling factor
                                orientation_intensity = min(1.0, abs(self.orientation) * 2.5)  # Increased from 1.5 to 2.5
                                
                                # Apply cubic falloff for dramatically more pronounced panning effect
                                if self.orientation > 0.0:  # Right side dominant
                                    # Create a "hole" in the left channel for strong right panning
                                    # Cubic falloff for extreme separation
                                    left_attenuation = orientation_intensity * orientation_intensity * orientation_intensity
                                    final_left_gain *= max(0.05, (1.0 - (left_attenuation * 0.95)))  # Almost silent but never complete zero
                                    
                                    # Significant boost to the dominant side for clear directional effect
                                    # More boost for moderate values, less for extremes (to avoid distortion)
                                    right_boost = min(1.8, 1.0 + orientation_intensity * 0.6)
                                    final_right_gain *= right_boost
                                else:  # Left side dominant  
                                    # Create a "hole" in the right channel for strong left panning
                                    # Cubic falloff for extreme separation
                                    right_attenuation = orientation_intensity * orientation_intensity * orientation_intensity
                                    final_right_gain *= max(0.05, (1.0 - (right_attenuation * 0.95)))  # Almost silent but never complete zero
                                    
                                    # Significant boost to the dominant side for clear directional effect
                                    left_boost = min(1.8, 1.0 + orientation_intensity * 0.6)
                                    final_left_gain *= left_boost
                                
                                # Mix with soft clipping protection
                                mixed = base_tone + noise * 0.5
                                # Soft clipping function (tanh approximation)
                                if mixed > 0.9:
                                    mixed = 0.9 + (mixed - 0.9) / (1.0 + abs(mixed - 0.9) * 5.0)
                                elif mixed < -0.9:
                                    mixed = -0.9 + (mixed + 0.9) / (1.0 + abs(mixed + 0.9) * 5.0)
                                
                                sample = mixed * 0.95 * 32767.0  # Final scaling with headroom
                                
                                samples[i, 0] = int(sample * final_left_gain)
                                samples[i, 1] = int(sample * final_right_gain)
                                
                                # Update phases with orientation-based frequency shift
                                # This creates a slight frequency difference between ears when panning
                                # which enhances the stereo image perception
                                freq_shift = 1.0 + (self.orientation * 0.04)  # Small frequency shift based on orientation
                                
                                # Apply frequency shift to create stereo widening effect
                                if self.orientation > 0:
                                    # When panning right, increase right frequency slightly
                                    tone1_phase += 2 * math.pi * 110 / self.sample_rate
                                    tone2_phase += 2 * math.pi * (146.83 * freq_shift) / self.sample_rate
                                    tone3_phase += 2 * math.pi * 73.42 / self.sample_rate
                                    self.phase_left += 2 * math.pi * 60.0 / self.sample_rate
                                    self.phase_right += 2 * math.pi * (60.1 * freq_shift) / self.sample_rate
                                else:
                                    # When panning left, increase left frequency slightly
                                    tone1_phase += 2 * math.pi * (110 * (2.0 - freq_shift)) / self.sample_rate
                                    tone2_phase += 2 * math.pi * 146.83 / self.sample_rate
                                    tone3_phase += 2 * math.pi * (73.42 * (2.0 - freq_shift)) / self.sample_rate
                                    self.phase_left += 2 * math.pi * (60.0 * (2.0 - freq_shift)) / self.sample_rate
                                    self.phase_right += 2 * math.pi * 60.1 / self.sample_rate
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