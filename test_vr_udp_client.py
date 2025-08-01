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
import math

# Global message order counter
message_order = 0

def send_vr_command(host='127.0.0.1', port=5000, head_pitch=0.0, head_pan=0.0, head_tilt=0.0, 
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

def continuous_send_test(rate_hz=25.0, duration_sec=30.0):
    """Send VR commands continuously at specified rate for testing monitoring"""
    print(f"Sending VR commands continuously at {rate_hz}Hz for {duration_sec} seconds...")
    print("This tests the monitoring system in the VR interface node")
    
    import time
    
    interval = 1.0 / rate_hz
    start_time = time.time()
    packet_count = 0
    
    try:
        while time.time() - start_time < duration_sec:
            loop_start = time.time()
            
            # Send a simple test command
            send_vr_command(
                head_pitch=0.1 * math.sin(packet_count * 0.1),
                head_pan=0.1 * math.cos(packet_count * 0.1), 
                head_tilt=0.0,
                base_state=0,
                base_angular=0,
                audio_volume=50,
                audio_orientation=0.0
            )
            
            packet_count += 1
            
            # Sleep to maintain the rate
            elapsed = time.time() - loop_start
            sleep_time = max(0, interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # Log every 5 seconds
            if packet_count % (int(rate_hz) * 5) == 0:
                actual_rate = packet_count / (time.time() - start_time)
                print(f"  Sent {packet_count} packets, actual rate: {actual_rate:.1f}Hz")
                
    except KeyboardInterrupt:
        print(f"\nStopped after sending {packet_count} packets")
    
    actual_duration = time.time() - start_time
    actual_rate = packet_count / actual_duration
    print(f"Test completed: {packet_count} packets in {actual_duration:.1f}s (avg rate: {actual_rate:.1f}Hz)")

def rate_test_menu():
    """Interactive menu for rate testing"""
    print("\nRate Testing Menu")
    print("1. Send at 25Hz for 30 seconds (normal rate)")
    print("2. Send at 10Hz for 20 seconds (slow rate)")
    print("3. Send at 50Hz for 15 seconds (fast rate)")
    print("4. Stop sending for 15 seconds (test error detection)")
    print("5. Custom rate and duration")
    print("6. Back to main menu")
    
    choice = input("Select option (1-6): ").strip()
    
    if choice == '1':
        continuous_send_test(25.0, 30.0)
    elif choice == '2':
        continuous_send_test(10.0, 20.0)
    elif choice == '3':
        continuous_send_test(50.0, 15.0)
    elif choice == '4':
        print("Stopping sends for 15 seconds to test error detection...")
        print("Check VR interface node logs for error messages")
        time.sleep(15)
        print("Resume testing or check logs")
    elif choice == '5':
        try:
            rate = float(input("Enter rate (Hz): "))
            duration = float(input("Enter duration (seconds): "))
            continuous_send_test(rate, duration)
        except ValueError:
            print("Invalid input")
    elif choice == '6':
        return
def interactive_mode():
    """Interactive mode for manual testing"""
    print("\nInteractive VR Command Mode")
    print("1. Send single command")
    print("2. Demo sequence")
    print("3. Rate testing menu")
    print("4. Quit")
    
    while True:
        try:
            choice = input("\nSelect option (1-4): ").strip()
            
            if choice == '1':
                manual_command_mode()
            elif choice == '2':
                demo_sequence()
            elif choice == '3':
                rate_test_menu()
            elif choice == '4':
                break
            else:
                print("Invalid choice")
                
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")

def manual_command_mode():
    """Manual command input mode"""
    print("\nManual Command Mode")
    print("Enter commands in the format: pitch pan tilt state angular volume orientation")
    print("Example: 1.0 0.5 -0.5 3 1 75 90.0")
    print("Type 'back' to return to main menu")
    
    while True:
        try:
            cmd = input("\nVR command> ").strip()
            
            if cmd.lower() == 'back':
                break
            elif cmd.lower() == 'help':
                print("Commands:")
                print("  pitch pan tilt state angular volume orientation - Send VR command")
                print("    pitch/pan/tilt: float values for head control")
                print("    state: int (0=idle, 1=push, 2=timing, 3=atomic)")
                print("    angular: int (-1=left, 0=forward, 1=right)")
                print("    volume: int (audio volume)")
                print("    orientation: float (audio orientation in degrees)")
                print("  back - Return to main menu")
                print("  help - Show this help")
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
            print("\nReturning to main menu...")
            break
        except Exception as e:
            print(f"Error: {e}")
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
    if len(sys.argv) > 1:
        if sys.argv[1] == 'demo':
            demo_sequence()
        elif sys.argv[1] == 'rate':
            rate_test_menu()
        elif sys.argv[1] in ['-h', '--help', 'help']:
            print("VR UDP Test Client")
            print("Usage:")
            print("  python test_vr_udp_client.py         - Interactive mode")
            print("  python test_vr_udp_client.py demo    - Run demo sequence")
            print("  python test_vr_udp_client.py rate    - Rate testing menu")
            print("  python test_vr_udp_client.py help    - Show this help")
        else:
            print(f"Unknown argument: {sys.argv[1]}")
            print("Use 'help' for usage information")
    else:
        interactive_mode()
