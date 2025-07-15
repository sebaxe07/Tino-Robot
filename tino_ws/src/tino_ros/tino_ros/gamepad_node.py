import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from evdev import InputDevice, ecodes, categorize
import threading
import time

def find_event_device_for_logitech():
    import evdev
    for path in evdev.list_devices():
        try:
            dev = evdev.InputDevice(path)
            if 'Logitech' in dev.name or 'F710' in dev.name:
                return path
        except:
            continue
    return None


class GamepadNode(Node):
    def __init__(self):
        super().__init__('gamepad_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'base_cmd_vel', 10)
        self.head_pub = self.create_publisher(Twist, 'head_cmd', 10)

        device_path = find_event_device_for_logitech()
        if not device_path:
            self.get_logger().warn("No Logitech event device found, will retry...")
        self.gamepad_device = device_path or '/dev/input/event0'  # fallback


        # Input mode setting
        self.x_input_mode = True  # Set to True for X input mode, False for D input
        self.x_input_range = (-32768, 32767)  # Typical X input range
        self.d_input_range = (0, 255)  # D input range
        self.neutral_zone_percent = 5  # Neutral zone as a percentage of total range

        # Initialize gamepad values
        self.bf = 0.0  # Base forward/backward
        self.bb = 0.0  # Base rotation
        self.hp = 0.0  # Head pitch
        self.hx = 511.0  # Head X
        self.hy = 511.0  # Head Y
        
        # Button states for leg commands
        self.leg_command = 0  # 0=idle, 1=little push (X), 2=forward only (Y), 3=finish cycle (B)
        
        # Pulse control for leg commands
        self.leg_command_pulse_count = 0  # Counter for pulse transmission
        self.leg_command_pulse_target = -1  # Target command to pulse (-1 means no pulse active)
        self.pulse_duration = 3  # Number of times to send the command
        
        # Pulse control for rotation (bumpers)
        self.bb_pulse_count = 0  # Counter for rotation pulse transmission
        self.bb_pulse_target = 0.0  # Target rotation value to pulse (0.0 means no pulse active)
        self.bb_pulse_duration = 3  # Number of times to send the rotation
        
        # Status flags
        self.is_running = True
        self.gamepad_connected = False
        
        # Set up publishing timer
        self.create_timer(0.04, self.publish_commands)  # 25Hz like original code
        
        # Start gamepad thread
        self.gamepad_thread = threading.Thread(target=self.read_gamepad)
        self.gamepad_thread.daemon = True
        self.gamepad_thread.start()
        
        self.get_logger().info(f"Gamepad node initialized, using device: {self.gamepad_device}")
    
    def trigger_leg_command_pulse(self, command):
        """Trigger a pulse for the specified leg command"""
        self.leg_command_pulse_target = command
        self.leg_command_pulse_count = 0
        self.leg_command = command
        self.get_logger().info(f"Triggering leg command pulse: {command}")
    
    def trigger_rotation_pulse(self, rotation_value):
        """Trigger a pulse for the specified rotation value with command 3"""
        self.bb_pulse_target = rotation_value
        self.bb_pulse_count = 0
        self.bb = rotation_value
        self.trigger_leg_command_pulse(3)  # Also trigger case 3
        self.get_logger().info(f"Triggering rotation pulse: {rotation_value} + Leg command pulse: 3")
    
    def normalize_input(self, value, axis_type):
        """Normalize X-input values to match the expected ranges in the code"""
        if not self.x_input_mode:
            return value  # No normalization needed for D-input
        
        x_min, x_max = self.x_input_range
        d_min, d_max = self.d_input_range
        
        # For axis inputs, normalize from X input range to D input range
        normalized = self.map_range(value, x_min, x_max, d_min, d_max)
        return int(normalized)
    
    def is_neutral(self, value, axis_type):
        """Check if the value is in the neutral position"""
        if not self.x_input_mode:
            # D-input neutral check (original code)
            return 120 <= value <= 130
        
        # For X-input, use a percentage-based neutral zone around 0
        x_min, x_max = self.x_input_range
        full_range = x_max - x_min
        neutral_range = full_range * (self.neutral_zone_percent / 100.0)
        return abs(value) < neutral_range

    def map_range(self, value, in_min, in_max, out_min, out_max):
        """Map a value from one range to another"""
        return out_min + (((value - in_min) / (in_max - in_min)) * (out_max - out_min))
    
    def read_gamepad(self):
        """Thread function to read gamepad input"""
        while self.is_running:
            try:
                gamepad = InputDevice(self.gamepad_device)
                self.gamepad_connected = True
                self.get_logger().info("Gamepad connected")
                
                # Main loop for reading gamepad events
                for event in gamepad.read_loop():
                    if not self.is_running:
                        break
                        
                    if event.code == 0 and event.type == 0:
                        continue
                        

                    # if event.type == ecodes.EV_ABS or event.type == ecodes.EV_KEY:
                    #     self.get_logger().info(
                    #         f"Event: type={event.type}, code={event.code} ({ecodes.ABS.get(event.code) or ecodes.KEY.get(event.code)}), value={event.value}"
                    #     )

                    # Process gamepad events
                    if event.type == ecodes.EV_ABS:
                        if event.code == 4 and event.type == 3:  # HY - Right stick Y
                            raw_value = event.value
                            
                            if self.is_neutral(raw_value, "right_y"):
                                self.hy = 511.0
                            else:
                                if self.x_input_mode:
                                    # Map from X-input range to our 0-1024 range
                                    self.hy = self.map_range(raw_value, self.x_input_range[0], self.x_input_range[1], 0, 1024)
                                else:
                                    # Original D-input code
                                    in_min = 130 if event.value < 120 else 0
                                    in_max = 255 if event.value < 120 else 120
                                    out_min = 512 if event.value < 120 else 0
                                    out_max = 1024 if event.value < 120 else 511
                                    self.hy = self.map_range(event.value, in_min, in_max, out_min, out_max)
                        elif event.code == 3 and event.type == 3: # HX - Right stick X
                            raw_value = event.value
                            
                            if self.is_neutral(raw_value, "right_x"):
                                self.hx = 511.0
                            else:
                                if self.x_input_mode:
                                    # Map from X-input range to our 0-1024 range
                                    self.hx = self.map_range(raw_value, self.x_input_range[0], self.x_input_range[1], 0, 1024)
                                else:
                                    # Original D-input code
                                    in_min = 130 if event.value < 120 else 0
                                    in_max = 255 if event.value < 120 else 120
                                    out_min = 512 if event.value < 120 else 0
                                    out_max = 1024 if event.value < 120 else 511
                                    self.hx = self.map_range(event.value, in_min, in_max, out_min, out_max)
                        elif event.code == 17:  # HP
                            self.hp = -335 if event.value < 0 else (335 if event.value > 0 else 0)
                            
                    # Face buttons for leg commands
                    elif event.type == ecodes.EV_KEY:
                        # Bumpers - send rotation + command 3 with pulse system
                        if event.code == 311:  # BB button right
                            if event.value == 1:  # Button pressed
                                self.trigger_rotation_pulse(-1.1)  # Trigger rotation pulse with command 3
                                self.get_logger().info("Right bumper pressed - Triggering rotation pulse: -1.1 + command 3")
                        elif event.code == 310:  # BB button left
                            if event.value == 1:  # Button pressed
                                self.trigger_rotation_pulse(1.1)  # Trigger rotation pulse with command 3
                                self.get_logger().info("Left bumper pressed - Triggering rotation pulse: 1.1 + command 3")
                        # Face buttons for leg commands
                        elif event.code == 307:  # X button
                            if event.value == 1:  # Button pressed
                                self.trigger_leg_command_pulse(1)  # Little push
                                self.get_logger().info("X button pressed - Leg command pulse: Little push (1)")
                        elif event.code == 308:  # Y button  
                            if event.value == 1:  # Button pressed
                                self.trigger_leg_command_pulse(2)  # Forward only
                                self.get_logger().info("Y button pressed - Leg command pulse: Forward only (2)")
                        elif event.code == 305:  # B button
                            if event.value == 1:  # Button pressed
                                self.trigger_leg_command_pulse(3)  # Finish cycle
                                self.get_logger().info("B button pressed - Leg command pulse: Finish cycle (3)")
                        elif event.code == 304:  # A button - Reset to idle
                            if event.value == 1:  # Button pressed
                                self.trigger_leg_command_pulse(0)  # Idle
                                self.get_logger().info("A button pressed - Leg command pulse: Idle (0)")


                gamepad.close()
                
            except FileNotFoundError:
                self.gamepad_connected = False
                self.get_logger().warning(f"Gamepad not found at {self.gamepad_device}. Retrying in 1 second...")
                time.sleep(1)
                
            except Exception as e:
                self.gamepad_connected = False
                self.get_logger().error(f"Gamepad error: {e}. Retrying in 1 second...")
                # print stack trace for debugging
                import traceback
                traceback.print_exc()
                time.sleep(1)
    
    def publish_commands(self):
        """Timer callback to publish command messages"""
        # Handle leg command pulse logic
        if self.leg_command_pulse_count < self.pulse_duration and self.leg_command_pulse_target != -1:
            # Continue sending the pulse command
            self.leg_command = self.leg_command_pulse_target
            self.leg_command_pulse_count += 1
            
            if self.leg_command_pulse_count >= self.pulse_duration:
                # Pulse complete, reset to idle
                self.leg_command = 0
                self.leg_command_pulse_target = -1  # Mark pulse as complete
                self.get_logger().info(f"Leg command pulse complete, returning to idle")
        
        # Handle rotation pulse logic (bumpers)
        if self.bb_pulse_count < self.bb_pulse_duration and self.bb_pulse_target != 0.0:
            # Continue sending the pulse rotation
            self.bb = self.bb_pulse_target
            self.bb_pulse_count += 1
            
            if self.bb_pulse_count >= self.bb_pulse_duration:
                # Pulse complete, reset to idle
                self.bb = 0.0
                self.bb_pulse_target = 0.0  # Mark pulse as complete
                self.get_logger().info(f"Rotation pulse complete, returning to idle")
        
        # Publish base velocity command
        base_cmd = Twist()
        base_cmd.linear.x = float(self.leg_command)  # Send leg command instead of forward/backward
        base_cmd.angular.z = float(self.bb)  # Rotation
        self.cmd_vel_pub.publish(base_cmd)
        
        # Publish head command
        head_cmd = Twist()
        head_cmd.angular.x = float(self.hp)  # Head pitch
        head_cmd.angular.y = float(self.hy - 511.0) / 511.0  # Head Y normalized to [-1, 1]
        head_cmd.angular.z = float(self.hx - 511.0) / 511.0  # Head X normalized to [-1, 1]
        self.head_pub.publish(head_cmd)
        
    def destroy_node(self):
        self.is_running = False
        if self.gamepad_thread.is_alive():
            self.gamepad_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GamepadNode()
    
    try:
        print("Gamepad node running. Press Ctrl+C to exit...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down gamepad node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()