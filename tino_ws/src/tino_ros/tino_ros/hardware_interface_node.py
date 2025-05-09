import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import threading
import time
import queue
import traceback  # Added for better error reporting

class HardwareInterfaceNode(Node):
    def __init__(self):
        super().__init__('hardware_interface_node')
        
        # Declare parameters
        self.declare_parameter('base_port', '/dev/ttyBASE')
        self.declare_parameter('leg_port', '/dev/ttyLEG')
        self.declare_parameter('head_port', '/dev/ttyHEAD')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('msg_repeat', 3)
        self.declare_parameter('debug_serial', False)
        
        # Print available serial ports for diagnostics
        self.list_available_serial_ports()
        
        # Initialize state
        self.bf = 0.0
        self.bb = 0.0
        self.hp = 0.0
        self.hx = 511.0
        self.hy = 511.0
        
        # Thread safety
        self.serial_lock = threading.RLock()
        self.command_queue = queue.Queue()
        self.is_running = True
        
        # Setup serial connections
        self.setup_serial_connections()
        
        # Subscribers
        self.base_sub = self.create_subscription(
            Twist, 'base_cmd_vel', self.base_callback, 10)
        self.head_sub = self.create_subscription(
            Twist, 'head_cmd', self.head_callback, 10)
        
        # Publisher for feedback
        self.feedback_pub = self.create_publisher(String, 'arduino_feedback', 10)
        
        # Start command sending timer - 25Hz
        self.create_timer(0.04, self.process_command_queue)
        
        self.get_logger().info("Hardware interface node initialized")
    
    def list_available_serial_ports(self):
        """List all available serial ports for diagnostics"""
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        self.get_logger().info(f"Available serial ports:")
        for port in ports:
            self.get_logger().info(f"  {port.device}: {port.description}")
    
    def setup_serial_connections(self):
        """Set up serial connections to Arduino hardware"""
        self.base_serial = None
        self.leg_serial = None
        self.head_serial = None
        
        # Try to connect to base Arduino
        base_port = self.get_parameter('base_port').value
        if base_port.lower() != 'none':
            try:
                self.get_logger().info(f"Attempting to connect to base Arduino on {base_port}")
                self.base_serial = serial.Serial(
                    port=base_port,
                    baudrate=self.get_parameter('baud_rate').value,
                    timeout=1
                )
                time.sleep(0.5)  # Shorter wait time
                self.base_serial.reset_input_buffer()
                self.base_serial.reset_output_buffer()
                self.get_logger().info(f"Successfully connected to base Arduino on {base_port}")
                
                # Start serial reader thread for base
                self.base_thread = threading.Thread(
                    target=self.read_serial_safe,
                    args=(self.base_serial, "BASE"),
                    daemon=True
                )
                self.base_thread.start()
            except Exception as e:
                self.get_logger().error(f"Failed to connect to base Arduino: {str(e)}")
                self.base_serial = None
        else:
            self.get_logger().info("Base Arduino port set to 'none', skipping connection")
            
        # Try to connect to leg Arduino
        leg_port = self.get_parameter('leg_port').value
        if leg_port.lower() != 'none':
            try:
                self.get_logger().info(f"Attempting to connect to leg Arduino on {leg_port}")
                self.leg_serial = serial.Serial(
                    port=leg_port,
                    baudrate=self.get_parameter('baud_rate').value,
                    timeout=1
                )
                time.sleep(0.5)  # Shorter wait time
                self.get_logger().info(f"Connected to leg Arduino on {leg_port}")
                
                # Start serial reader thread for leg
                self.leg_thread = threading.Thread(
                    target=self.read_serial_safe,
                    args=(self.leg_serial, "LEG"),
                    daemon=True
                )
                self.leg_thread.start()
            except Exception as e:
                self.get_logger().error(f"Failed to connect to leg Arduino: {e}")
                self.leg_serial = None
        else:
            self.get_logger().info("Leg Arduino port set to 'none', skipping connection")
            
        # Try to connect to head Arduino
        head_port = self.get_parameter('head_port').value
        if head_port.lower() != 'none':
            try:
                self.get_logger().info(f"Attempting to connect to head Arduino on {head_port}")
                self.head_serial = serial.Serial(
                    port=head_port,
                    baudrate=self.get_parameter('baud_rate').value,
                    timeout=1
                )
                time.sleep(0.5)  # Shorter wait time
                self.get_logger().info(f"Connected to head Arduino on {head_port}")
                
                # Start serial reader thread for head
                self.head_thread = threading.Thread(
                    target=self.read_serial_safe,
                    args=(self.head_serial, "HEAD"),
                    daemon=True
                )
                self.head_thread.start()
            except Exception as e:
                self.get_logger().error(f"Failed to connect to head Arduino: {e}")
                self.head_serial = None
        else:
            self.get_logger().info("Head Arduino port set to 'none', skipping connection")
        
        # Log summary of connections
        self.get_logger().info(f"Arduino connections: Base: {'Connected' if self.base_serial else 'Disconnected'}, " +
                             f"Leg: {'Connected' if self.leg_serial else 'Disconnected'}, " +
                             f"Head: {'Connected' if self.head_serial else 'Disconnected'}")
    
    def base_callback(self, msg):
        """Process base movement commands"""
        # Limit values to reasonable ranges
        self.bf = max(min(msg.linear.x, 25.0), -25.0)  # Limit max speed
        self.bb = max(min(msg.angular.z, 1.0), -1.0)   # Limit max angular velocity

        # Generate command and add to queue
        self.command_queue.put(self.generate_command())

    def head_callback(self, msg):
        """Process head movement commands"""
        # The RaspberryCommunication.cpp expects:
        # HF = Head Forward/backward (angular.x)
        # HP = Head Roll (angular.y) 
        # HX = Head Yaw (angular.z)
        
        # Swap Y and Z axes to correct the inverted movements
        self.hp = msg.angular.x  # Head Roll (pitch) comes from angular.y
        self.hx = 511.0 + (msg.angular.y * 511.0)  # Head Yaw (left/right)
        self.hy = 511.0 + (msg.angular.z * 511.0)  # Head Forward/backward
        
        # Generate command and add to queue
        self.command_queue.put(self.generate_command())

    def generate_command(self):
        """Generate command string from current state"""
        # Format HP (not HF) as an integer to ensure exact values
        # The Arduino expects HP as the key for head forward movement
        hp_int = int(self.hp)  # Convert to integer
        return f"BF:{self.bf:.2f}_BB:{self.bb:.2f}_HP:{hp_int}_HX:{self.hx:.2f}_HY:{self.hy:.2f}\n"

    def process_command_queue(self):
        """Process commands from queue and send to connected Arduinos"""
        try:
            # Get latest command (clear queue first to only keep latest)
            latest_cmd = None
            while not self.command_queue.empty():
                latest_cmd = self.command_queue.get_nowait()

            # If no commands in queue, generate one from current state
            if latest_cmd is None:
                latest_cmd = self.generate_command()

            self.send_command_to_arduino(latest_cmd)
        except Exception as e:
            self.get_logger().error(f"Error in process_command_queue: {str(e)}")
            traceback.print_exc()

    def send_command_to_arduino(self, command_str):
        """Send command to all connected Arduinos with proper error handling"""
        # Log command only when it contains significant movement commands
        significant_movement = False
        
        # Check if any value is significantly different from default/neutral
        if abs(self.bf) > 0.5 or abs(self.bb) > 0.1:  # Base movement
            significant_movement = True
        elif abs(self.hp) > 5.0 or abs(self.hx - 511.0) > 5.0 or abs(self.hy - 511.0) > 5.0:  # Head movement
            significant_movement = True
            
        if significant_movement and self.get_parameter('debug_serial').value:
            self.get_logger().info(f"Sending command: {command_str.strip()}")
        
        msg_repeat = self.get_parameter('msg_repeat').value
        
        # Use lock to ensure thread safety
        with self.serial_lock:
            # Send to base Arduino if connected
            if self.base_serial:
                try:
                    for _ in range(msg_repeat):
                        self.base_serial.write(command_str.encode('utf-8'))
                        self.base_serial.flush()
                        time.sleep(0.001)
                except Exception as e:
                    self.get_logger().error(f"Error writing to base Arduino: {e}")
            
            # Send to leg Arduino if connected
            if self.leg_serial:
                try:
                    for _ in range(msg_repeat):
                        self.leg_serial.write(command_str.encode('utf-8'))
                        self.leg_serial.flush()
                        time.sleep(0.001)
                except Exception as e:
                    self.get_logger().error(f"Error writing to leg Arduino: {e}")
            
            # Send to head Arduino if connected
            if self.head_serial:
                try:
                    for _ in range(msg_repeat):
                        self.head_serial.write(command_str.encode('utf-8'))
                        self.head_serial.flush()
                        time.sleep(0.001)
                except Exception as e:
                    self.get_logger().error(f"Error writing to head Arduino: {e}")

    def read_serial_safe(self, ser, name):
        """Thread function to read from serial port with robust error handling"""
        self.get_logger().info(f"Starting serial reader thread for {name}")
        buffer = ""
        
        while self.is_running and ser:
            try:
                # Check if there's data available to read
                if ser.in_waiting > 0:
                    try:
                        # Read one byte at a time to avoid blocking
                        byte = ser.read(1)
                        if byte:  # Only process if we actually got a byte
                            char = byte.decode('utf-8')
                            
                            # Add to buffer or process complete line
                            if char == '\n':
                                if buffer:
                                    # Always publish to feedback topic
                                    msg = String()
                                    msg.data = f"[{name}] {buffer}"
                                    self.feedback_pub.publish(msg)
                                    
                                    # Only log to console if not from HEAD Arduino
                                    if name != "HEAD":
                                        self.get_logger().info(f"[{name} Arduino] {buffer}")
                                    buffer = ""  # Reset buffer after processing
                            else:
                                buffer += char
                    except UnicodeDecodeError:
                        self.get_logger().warning(f"Received data from {name} that couldn't be decoded as UTF-8")
                        buffer = ""  # Reset buffer on decode error
                    except Exception as e:
                        self.get_logger().error(f"Error reading from {name} Arduino: {str(e)}")
                        buffer = ""  # Reset buffer on general error
                        time.sleep(0.1)  # Short delay to prevent CPU hammering
                else:
                    # No data available, small sleep to prevent CPU hogging
                    time.sleep(0.01)  # Longer sleep time to reduce CPU usage
            except Exception as e:
                self.get_logger().error(f"Error in reader thread for {name}: {str(e)}")
                time.sleep(0.5)  # Longer delay before trying again
                buffer = ""  # Reset buffer after error
    
    def close_connections(self):
        """Safely close all serial connections"""
        with self.serial_lock:
            # Close base serial
            if hasattr(self, 'base_serial') and self.base_serial:
                try:
                    self.base_serial.close()
                    self.base_serial = None
                except Exception as e:
                    self.get_logger().error(f"Error closing base serial: {str(e)}")
            
            # Close leg serial
            if hasattr(self, 'leg_serial') and self.leg_serial:
                try:
                    self.leg_serial.close()
                    self.leg_serial = None
                except Exception as e:
                    self.get_logger().error(f"Error closing leg serial: {str(e)}")
            
            # Close head serial
            if hasattr(self, 'head_serial') and self.head_serial:
                try:
                    self.head_serial.close()
                    self.head_serial = None
                except Exception as e:
                    self.get_logger().error(f"Error closing head serial: {str(e)}")
    
    def destroy_node(self):
        """Clean up resources when node is shutting down"""
        self.get_logger().info("Shutting down hardware interface node...")
        
        # Signal threads to stop
        self.is_running = False
        time.sleep(0.1)  # Give threads a moment to notice
        
        # Send stop command before closing
        try:
            self.bf = 0.0
            self.bb = 0.0
            self.hp = 0.0
            stop_cmd = self.generate_command()
            self.send_command_to_arduino(stop_cmd)
            time.sleep(0.1)  # Give it a moment to process
        except Exception as e:
            self.get_logger().error(f"Error sending stop command: {e}")
        
        # Close all serial connections
        self.close_connections()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterfaceNode()
    
    try:
        print("Hardware interface node running. Press Ctrl+C to exit...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Received keyboard interrupt...")
    except Exception as e:
        print(f"Unexpected error: {str(e)}")
        traceback.print_exc()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
