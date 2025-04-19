import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import threading

class HardwareInterfaceNode(Node):
    def __init__(self):
        super().__init__('hardware_interface_node')
        
        # Declare parameters
        self.declare_parameter('base_port', '/dev/ttyUSB0_BASE')
        self.declare_parameter('leg_port', '/dev/ttyUSB0_LEG')
        self.declare_parameter('head_port', '/dev/ttyUSB0_HEAD') 
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('msg_repeat', 3)
        
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
        
        # Start serial reader threads
        self.is_running = True
        if self.base_serial:
            self.base_thread = threading.Thread(target=self.read_serial, 
                                              args=(self.base_serial, "BASE"))
            self.base_thread.daemon = True
            self.base_thread.start()
            
        if self.leg_serial:
            self.leg_thread = threading.Thread(target=self.read_serial, 
                                             args=(self.leg_serial, "LEG"))
            self.leg_thread.daemon = True
            self.leg_thread.start()
            
        if self.head_serial:
            self.head_thread = threading.Thread(target=self.read_serial, 
                                              args=(self.head_serial, "HEAD"))
            self.head_thread.daemon = True
            self.head_thread.start()
        
        # Command sending timer
        self.create_timer(0.04, self.send_commands)  # 25Hz like original
        
        self.get_logger().info("Hardware interface node initialized")
    
    def setup_serial_connections(self):
        """Set up serial connections to Arduino hardware"""
        self.base_serial = None
        self.leg_serial = None
        self.head_serial = None
        
        try:
            base_port = self.get_parameter('base_port').value
            self.base_serial = serial.Serial(base_port, 
                                           self.get_parameter('baud_rate').value, 
                                           timeout=1)
            self.get_logger().info(f"Connected to base Arduino on {base_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to base Arduino: {e}")
            
        try:
            leg_port = self.get_parameter('leg_port').value
            self.leg_serial = serial.Serial(leg_port, 
                                          self.get_parameter('baud_rate').value, 
                                          timeout=1)
            self.get_logger().info(f"Connected to leg Arduino on {leg_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to leg Arduino: {e}")
            
        try:
            head_port = self.get_parameter('head_port').value
            self.head_serial = serial.Serial(head_port, 
                                           self.get_parameter('baud_rate').value, 
                                           timeout=1)
            self.get_logger().info(f"Connected to head Arduino on {head_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to head Arduino: {e}")
    
    def base_callback(self, msg):
        """Process base movement commands"""
        self.bf = msg.linear.x
        self.bb = msg.angular.z
    
    def head_callback(self, msg):
        """Process head movement commands"""
        self.hp = msg.angular.x
        self.hy = 511.0 + (msg.angular.y * 511.0)
        self.hx = 511.0 + (msg.angular.z * 511.0)
    
    def send_commands(self):
        """Send commands to all Arduino boards"""
        if not (self.base_serial or self.leg_serial or self.head_serial):
            return
            
        # Format command string like in original code
        command = (f"BF:{self.bf:.2f}_BB:{self.bb:.2f}_HP:{self.hp:.2f}"
                  f"_HX:{self.hx:.2f}_HY:{self.hy:.2f}\n")
        
        # Send to all connected Arduino boards
        for _ in range(self.get_parameter('msg_repeat').value):
            if self.base_serial:
                try:
                    self.base_serial.write(command.encode('utf-8'))
                except Exception as e:
                    self.get_logger().error(f"Error writing to base Arduino: {e}")
            
            if self.leg_serial:
                try:
                    self.leg_serial.write(command.encode('utf-8'))
                except Exception as e:
                    self.get_logger().error(f"Error writing to leg Arduino: {e}")
            
            if self.head_serial:
                try:
                    self.head_serial.write(command.encode('utf-8'))
                except Exception as e:
                    self.get_logger().error(f"Error writing to head Arduino: {e}")
    
    def read_serial(self, ser, name):
        """Thread function to read from serial port"""
        while self.is_running and ser:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').rstrip()
                    msg = String()
                    msg.data = f"[{name}] {line}"
                    self.feedback_pub.publish(msg)
                    print(f"[{name}] {line}")
            except Exception as e:
                self.get_logger().error(f"Error reading from {name} Arduino: {e}")
    
    def destroy_node(self):
        """Clean up resources when node is shutting down"""
        self.is_running = False
        
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