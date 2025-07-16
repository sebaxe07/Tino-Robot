#!/usr/bin/env python3
"""
Test UDP client for sending VR data to the VR interface node.
This demonstrates the expected message format.

Message format: float,float,float,int,int,int,float,int
- First 3 floats: head_pitch, head_pan, head_tilt
- Next 2 ints: base_state (0,1,2,3), base_angular (-1,0,1)
- Next int + float: audio_volume (int), audio_orientation (float)
- Last int: message_order (32-bit int for ordering)
"""

import socket
import struct
import time
import sys

# Global message order counter
message_order = 0

def send_vr_command(host='127.0.0.1', port=5005, head_pitch=0.0, head_pan=0.0, head_tilt=0.0, 
                   base_state=0, base_angular=0, audio_volume=0, audio_orientation=0.0):
    """Send a single VR command via UDP"""
    global message_order
    
    try:
        # Create UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Pack the data: 3 floats + 2 ints + 1 int + 1 float + 1 int = 32 bytes
        data = struct.pack('fffiiifi', head_pitch, head_pan, head_tilt, base_state, base_angular, 
                          audio_volume, audio_orientation, message_order)
        
        # Send the data
        sock.sendto(data, (host, port))
        sock.close()
        
        print(f"Sent VR command #{message_order}: pitch={head_pitch:.2f}, pan={head_pan:.2f}, tilt={head_tilt:.2f}, "
              f"state={base_state}, angular={base_angular}, vol={audio_volume}, orient={audio_orientation:.2f}")
        
        # Increment message order (with 32-bit overflow handling)
        message_order = (message_order + 1) % (2**31)
        
    except Exception as e:
        print(f"Error sending VR command: {e}")

def demo_sequence():
    """Demonstrate a sequence of VR commands"""
    print("Starting VR command demonstration...")
    print("Make sure the VR interface node is running!")
    time.sleep(2)
    
    # Reset to idle
    print("\n1. Reset to idle state")
    send_vr_command(head_pitch=0.0, head_pan=0.0, head_tilt=0.0, base_state=0, base_angular=0, 
                   audio_volume=50, audio_orientation=0.0)
    time.sleep(2)
    
    # Test head movements
    print("\n2. Testing head movements")
    send_vr_command(head_pitch=1.0, head_pan=0.0, head_tilt=0.0, base_state=0, base_angular=0,
                   audio_volume=60, audio_orientation=0.0)  # Look down
    time.sleep(1)
    send_vr_command(head_pitch=0.0, head_pan=1.0, head_tilt=0.0, base_state=0, base_angular=0,
                   audio_volume=70, audio_orientation=90.0)  # Look right
    time.sleep(1)
    send_vr_command(head_pitch=0.0, head_pan=0.0, head_tilt=1.0, base_state=0, base_angular=0,
                   audio_volume=80, audio_orientation=180.0)  # Tilt
    time.sleep(1)
    send_vr_command(head_pitch=0.0, head_pan=0.0, head_tilt=0.0, base_state=0, base_angular=0,
                   audio_volume=50, audio_orientation=0.0)  # Center
    time.sleep(2)
    
    # Test base movements
    print("\n3. Testing base movements")
    
    # Little push (case 1)
    print("   - Little push movement")
    send_vr_command(head_pitch=0.0, head_pan=0.0, head_tilt=0.0, base_state=1, base_angular=0,
                   audio_volume=60, audio_orientation=0.0)
    time.sleep(3)
    
    # Timing cycle (case 2)
    print("   - Timing cycle")
    send_vr_command(head_pitch=0.0, head_pan=0.0, head_tilt=0.0, base_state=2, base_angular=0,
                   audio_volume=70, audio_orientation=45.0)
    time.sleep(3)
    
    # Atomic movements (case 3)
    print("   - Forward movement")
    send_vr_command(head_pitch=0.0, head_pan=0.0, head_tilt=0.0, base_state=3, base_angular=0,
                   audio_volume=80, audio_orientation=0.0)
    time.sleep(3)
    
    print("   - Right rotation")
    send_vr_command(head_pitch=0.0, head_pan=0.0, head_tilt=0.0, base_state=3, base_angular=1,
                   audio_volume=90, audio_orientation=90.0)
    time.sleep(3)
    
    print("   - Left rotation")
    send_vr_command(head_pitch=0.0, head_pan=0.0, head_tilt=0.0, base_state=3, base_angular=-1,
                   audio_volume=85, audio_orientation=270.0)
    time.sleep(3)
    
    # Test audio variations
    print("\n4. Testing audio variations")
    for i in range(5):
        volume = 50 + i * 10
        orientation = i * 72.0  # 0, 72, 144, 216, 288 degrees
        send_vr_command(head_pitch=0.0, head_pan=0.0, head_tilt=0.0, base_state=0, base_angular=0,
                       audio_volume=volume, audio_orientation=orientation)
        time.sleep(0.5)
    
    # Back to idle
    print("\n5. Back to idle")
    send_vr_command(head_pitch=0.0, head_pan=0.0, head_tilt=0.0, base_state=0, base_angular=0,
                   audio_volume=50, audio_orientation=0.0)
    
    print("\nDemo sequence completed!")

def interactive_mode():
    """Interactive mode for manual testing"""
    print("\nInteractive VR Command Mode")
    print("Enter commands in the format: pitch pan tilt state angular volume orientation")
    print("Example: 1.0 0.5 -0.5 3 1 75 90.0")
    print("Type 'demo' for demonstration sequence")
    print("Type 'quit' to exit")
    
    while True:
        try:
            cmd = input("\nVR command> ").strip()
            
            if cmd.lower() == 'quit':
                break
            elif cmd.lower() == 'demo':
                demo_sequence()
                continue
            elif cmd.lower() == 'help':
                print("Commands:")
                print("  pitch pan tilt state angular volume orientation - Send VR command")
                print("    pitch/pan/tilt: float values for head control")
                print("    state: int (0=idle, 1=push, 2=timing, 3=atomic)")
                print("    angular: int (-1=left, 0=forward, 1=right)")
                print("    volume: int (audio volume)")
                print("    orientation: float (audio orientation in degrees)")
                print("  demo - Run demonstration sequence") 
                print("  help - Show this help")
                print("  quit - Exit")
                continue
                
            # Parse command
            parts = cmd.split()
            if len(parts) != 7:
                print("Error: Expected 7 values (pitch pan tilt state angular volume orientation)")
                continue
                
            pitch = float(parts[0])
            pan = float(parts[1])
            tilt = float(parts[2])
            state = int(parts[3])
            angular = int(parts[4])
            volume = int(parts[5])
            orientation = float(parts[6])
            
            send_vr_command(head_pitch=pitch, head_pan=pan, head_tilt=tilt, 
                          base_state=state, base_angular=angular,
                          audio_volume=volume, audio_orientation=orientation)
            
        except ValueError:
            print("Error: Invalid number format")
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == 'demo':
        demo_sequence()
    else:
        interactive_mode()
