#!/usr/bin/env python3
"""
Test UDP receiver for skeleton data from the VR interface node.
This demonstrates how to parse the skeleton data packet.

Skeleton packet format: 
- 1 int (message_order) + 51 floats (17 joints * 3 coordinates each)
- Total: 4 + (51 * 4) = 208 bytes
"""

import socket
import struct
import time
import sys

def receive_skeleton_data(host='127.0.0.1', port=5007):
    """Receive and parse skeleton data via UDP"""
    try:
        # Create UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((host, port))
        sock.settimeout(1.0)
        
        print(f"Listening for skeleton data on {host}:{port}")
        print("Expected packet size: 208 bytes (1 int + 51 floats)")
        print("Joint order (COCO format):")
        
        joint_names = [
            'nose', 'left_eye', 'right_eye', 'left_ear', 'right_ear',
            'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
            'left_wrist', 'right_wrist', 'left_hip', 'right_hip',
            'left_knee', 'right_knee', 'left_ankle', 'right_ankle'
        ]
        
        for i, name in enumerate(joint_names):
            print(f"  {i:2d}: {name}")
        print()
        
        packet_count = 0
        
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                packet_count += 1
                
                expected_size = 4 + (51 * 4)  # 1 int + 51 floats = 208 bytes
                
                if len(data) == expected_size:
                    # Unpack: 1 int + 51 floats
                    values = struct.unpack('i51f', data)
                    message_order = values[0]
                    skeleton_data = values[1:]  # 51 floats
                    
                    print(f"Packet #{packet_count} (order: {message_order}) from {addr}")
                    print(f"Data size: {len(data)} bytes (correct)")
                    
                    # Parse skeleton joints
                    joints_detected = 0
                    for i in range(17):  # 17 joints
                        x = skeleton_data[i*3]
                        y = skeleton_data[i*3 + 1] 
                        z = skeleton_data[i*3 + 2]
                        
                        # Check if joint is detected (non-zero)
                        if x != 0.0 or y != 0.0 or z != 0.0:
                            joints_detected += 1
                            print(f"  {joint_names[i]:15s}: ({x:6.3f}, {y:6.3f}, {z:6.3f})")
                    
                    print(f"  Joints detected: {joints_detected}/17")
                    if joints_detected < 17:
                        missing = 17 - joints_detected
                        print(f"  Missing joints: {missing} (set to 0,0,0)")
                    print()
                    
                else:
                    print(f"Packet #{packet_count}: Invalid size {len(data)} bytes, expected {expected_size}")
                    
            except socket.timeout:
                # Normal timeout, continue listening
                continue
            except KeyboardInterrupt:
                print("\nExiting...")
                break
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()

def show_help():
    print("Skeleton Data Receiver")
    print("Usage: python test_skeleton_udp_receiver.py [port]")
    print("")
    print("Options:")
    print("  port    UDP port to listen on (default: 5007)")
    print("")
    print("Expected data format:")
    print("  - 1 int: message order")
    print("  - 51 floats: 17 joints * 3 coordinates (x,y,z) each")
    print("  - Total packet size: 208 bytes")
    print("")
    print("Make sure the VR interface node is running and sending skeleton data!")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] in ['-h', '--help', 'help']:
            show_help()
            sys.exit(0)
        try:
            port = int(sys.argv[1])
        except ValueError:
            print("Error: Port must be a number")
            show_help()
            sys.exit(1)
    else:
        port = 5007
    
    receive_skeleton_data(port=port)
