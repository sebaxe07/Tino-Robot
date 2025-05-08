import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import threading
import time

class HardwareInterfaceNode(Node):
    def __init__(self):
        super().__init__('hardware_interface_node')
        
        # Declare parameters
        self.declare_parameter('base_port', '/dev/ttyACM0')  # Updated to new port
        self.declare_parameter('leg_port', 'none')
        self.declare_parameter('head_port', 'none')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('msg_repeat', 3)
        self.declare_parameter('debug_serial', True)  # New parameter to enable serial debugging
        
        # Print available serial ports for diagnostics
        self.list_available_serial_ports()
        
        # Setup serial connections
        self.setup_serial_connections()
        
        # Subscribers
        self.base_sub = self.create_subscription(
            Twist, 'base_cmd_vel', self.base_callback, 10)
        self.head_sub = self.create_subscription(
            Twist, 'head_cmd', self.head_callback, 10)
        
        # Publisher for feedback
        self.feedback_pub = self.create_publisher(String, 'arduino_feedback', 10)
        
        # Initialize state
        self.bf = 0.0
        self.bb = 0.0
        self.hp = 0.0
        self.hx = 511.0
        self.hy = 511.0
        
        # Last command time tracking for heartbeats
        self.last_command_sent = time.time()
        
        # Start serial reader threads
        self.is_running = True
        if self.base_serial:
            self.base_thread = threading.Thread(target=self.read_serial, 
                                              args=(self.base_serial, "BASE"))
            self.base_thread.daemon = True
            self.base_thread.start()
            
            # Send initial ping to check if Arduino is responsive
            self.get_logger().info("Sending initial ping to Arduino...")
            self.send_ping()
            
        # Start command sending timer - 25Hz
        self.create_timer(0.04, self.send_commands)
        
        # Start heartbeat timer - every 1 second
        self.create_timer(1.0, self.send_heartbeat)
        
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
                # Wait for Arduino to reset after connection
                time.sleep(2)
                self.base_serial.reset_input_buffer()
                self.base_serial.reset_output_buffer()  # Also reset output buffer
                self.get_logger().info(f"Successfully connected to base Arduino on {base_port}")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to base Arduino: {str(e)}")
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
                self.get_logger().info(f"Connected to leg Arduino on {leg_port}")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to leg Arduino: {e}")
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
                self.get_logger().info(f"Connected to head Arduino on {head_port}")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to head Arduino: {e}")
        else:
            self.get_logger().info("Head Arduino port set to 'none', skipping connection")
    
    def base_callback(self, msg):
        """Process base movement commands"""
        # Limit values to reasonable ranges
        self.bf = max(min(msg.linear.x, 25.0), -25.0)  # Limit max speed
        self.bb = max(min(msg.angular.z, 1.0), -1.0)   # Limit max angular velocity
        
        # Add debug log when receiving movement commands
        if abs(self.bf) > 0.01 or abs(self.bb) > 0.01:
            self.get_logger().info(f"MOVEMENT DEBUG: Received base movement - linear: {self.bf:.2f}, angular: {self.bb:.2f}")
    
    def head_callback(self, msg):
        """Process head movement commands"""
        self.hp = msg.angular.x
        self.hy = 511.0 + (msg.angular.y * 511.0)
        self.hx = 511.0 + (msg.angular.z * 511.0)
    
    def send_ping(self):
        """Send a ping command to check if Arduino is responsive"""
        if self.base_serial:
            try:
                ping_cmd = "PING:1\n"
                self.base_serial.write(ping_cmd.encode('utf-8'))
                self.base_serial.flush()
                self.get_logger().info("Ping sent to Arduino")
            except Exception as e:
                self.get_logger().error(f"Error sending ping: {e}")
    
    def send_heartbeat(self):
        """Send a heartbeat message to keep the connection alive"""
        # Send a message with zero movement if no command has been sent recently
        current_time = time.time()
        if current_time - self.last_command_sent > 0.8:  # If no command for 0.8 seconds
            if abs(self.bf) < 0.01 and abs(self.bb) < 0.01:  # Only if we're not actively moving
                if self.base_serial:
                    try:
                        heartbeat_cmd = "BF:0.00_BB:0.00_HB:1\n"  # HB is heartbeat flag
                        self.base_serial.write(heartbeat_cmd.encode('utf-8'))
                        self.base_serial.flush()
                        if self.get_parameter('debug_serial').value:
                            self.get_logger().info("Heartbeat sent to Arduino")
                        self.last_command_sent = current_time
                    except Exception as e:
                        self.get_logger().error(f"Error sending heartbeat: {e}")
                        self.reconnect_base()
    
    def send_commands(self):
        """Send commands to all Arduino boards"""
        if not (self.base_serial or self.leg_serial or self.head_serial):
            return
            
        # Only send base commands if we have a base serial connection
        if self.base_serial:
            # Format command string - only include what the base Arduino needs
            base_command = f"BF:{self.bf:.2f}_BB:{self.bb:.2f}\n"
            
            # Add debug log when sending commands to Arduino with non-zero movement
            if abs(self.bf) > 0.01 or abs(self.bb) > 0.01:
                self.get_logger().info(f"MOVEMENT DEBUG: Sending to Arduino: {base_command.strip()}")
            
            try:
                # Send command multiple times for reliability
                for _ in range(self.get_parameter('msg_repeat').value):
                    self.base_serial.write(base_command.encode('utf-8'))
                    self.base_serial.flush()  # Ensure data is sent immediately
                    time.sleep(0.001)  # Small delay between repeats
                
                # Update last command time
                self.last_command_sent = time.time()
            except Exception as e:
                self.get_logger().error(f"Error writing to base Arduino: {e}")
                # Try to reconnect on error
                self.reconnect_base()
    
    def reconnect_base(self):
        """Try to reconnect to the base Arduino if connection is lost"""
        try:
            if self.base_serial:
                self.base_serial.close()
            
            base_port = self.get_parameter('base_port').value
            self.get_logger().info(f"Attempting to reconnect to base Arduino on {base_port}")
            self.base_serial = serial.Serial(
                port=base_port,
                baudrate=self.get_parameter('baud_rate').value,
                timeout=1
            )
            time.sleep(2)  # Wait for Arduino reset
            self.base_serial.reset_input_buffer()  # Clear any pending input
            self.base_serial.reset_output_buffer() # Clear any pending output
            self.get_logger().info("Reconnected to base Arduino")
            
            # Send a ping after reconnecting
            self.send_ping()
        except Exception as e:
            self.get_logger().error(f"Failed to reconnect to base Arduino: {e}")
    
    def read_serial(self, ser, name):
        """Thread function to read from serial port"""
        self.get_logger().info(f"Starting serial reader thread for {name}")
        buffer = ""
        
        while self.is_running and ser:
            try:
                # Check if there's data available to read
                if ser.in_waiting > 0:
                    try:
                        # Read one byte at a time to avoid blocking
                        byte = ser.read(1)
                        char = byte.decode('utf-8')
                        
                        # Add to buffer or process complete line
                        if char == '\n':
                            if buffer:
                                # Create message and publish
                                msg = String()
                                msg.data = f"[{name}] {buffer}"
                                self.feedback_pub.publish(msg)
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
                else:
                    # No data available, small sleep to prevent CPU hogging
                    time.sleep(0.005)  # Shorter sleep time for better responsiveness
            except Exception as e:
                self.get_logger().error(f"Error in reader thread for {name}: {str(e)}")
                # Try to reconnect if it's the base Arduino
                if name == "BASE":
                    self.reconnect_base()
                # Small delay before trying again
                time.sleep(1.0)
                buffer = ""  # Reset buffer after reconnect attempt
    
    def destroy_node(self):
        """Clean up resources when node is shutting down"""
        self.is_running = False
        
        # Send stop command before closing
        if hasattr(self, 'base_serial') and self.base_serial:
            try:
                self.bf = 0.0
                self.bb = 0.0
                stop_cmd = "BF:0.00_BB:0.00_STOP:1\n"
                self.base_serial.write(stop_cmd.encode('utf-8'))
                self.base_serial.flush()
                self.get_logger().info("Sent stop command to Arduino")
                time.sleep(0.1)  # Give it a moment to process
            except Exception as e:
                self.get_logger().error(f"Error sending stop command: {e}")
        
        # Close serial connections
        if hasattr(self, 'base_serial') and self.base_serial:
            self.base_serial.close()
        if hasattr(self, 'leg_serial') and self.leg_serial:
            self.leg_serial.close()
        if hasattr(self, 'head_serial') and self.head_serial:
            self.head_serial.close()
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterfaceNode()
    
    try:
        print("Hardware interface node running. Press Ctrl+C to exit...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down hardware interface node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
