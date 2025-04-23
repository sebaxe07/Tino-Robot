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
                        if event.code == 1:  # BF button / Left stick Y-axis
                            raw_value = event.value
                            
                            if self.is_neutral(raw_value, "left_y"):
                                self.bf = 0.0
                            else:
                                # In X-input mode, the Y axis is inverted (-32768 is down/forward, +32767 is up/backward)
                                # So we need to invert the sign for consistent behavior
                                if self.x_input_mode:
                                    normalized = self.normalize_input(raw_value, "left_y")
                                    # Forward motion (negative in X-input) maps to positive in our system
                                    if raw_value < 0:
                                        self.bf = self.map_range(normalized, self.d_input_range[0], 120, 15, 0)
                                    else:
                                        self.bf = self.map_range(normalized, 130, self.d_input_range[1], 0, -15)
                                else:
                                    # Original D-input code
                                    self.bf = self.map_range(event.value, 130, 255, 0, 0) if event.value > 130 else self.map_range(event.value, 0, 120, 15, 0)
                        
                        elif event.code == 4 and event.type == 3:  # HY - Right stick Y
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
                                self.hy = 511.0
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
                            print(f"HP button pressed: {event.value}")
                            self.hp = -335 if event.value < 0 else (335 if event.value > 0 else 0)
                            
                    # Bumpers
                    elif event.code == 311:  # BB button right
                        self.bb = -1.1 if event.value == 1 else 0.0
                    elif event.code == 310:  # BB button left
                        self.bb = 1.1 if event.value == 1 else 0.0


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
        # Publish base velocity command
        base_cmd = Twist()
        base_cmd.linear.x = float(self.bf)  # Forward/backward
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