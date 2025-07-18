#!/usr/bin/env python3

import socket
import sys

def udp_receiver(port=5000):
    """
    Simple UDP receiver that listens for incoming data on the specified port.
    
    Args:
        port (int): Port number to listen on (default: 5000)
    """
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        # Bind to all available interfaces on the specified port
        server_address = ('0.0.0.0', port)
        sock.bind(server_address)
        print(f"UDP receiver listening on port {port}")
        print("Press Ctrl+C to stop")
        print("-" * 50)
        
        while True:
            # Wait for incoming data
            data, client_address = sock.recvfrom(4096)  # Buffer size of 4096 bytes
            
            # Print received data
            print(f"Received from {client_address[0]}:{client_address[1]}")
            print(f"Data length: {len(data)} bytes")
            
            # Try to decode as UTF-8 text first
            try:
                decoded_data = data.decode('utf-8')
                print(f"Text data: {decoded_data}")
            except UnicodeDecodeError:
                # If not valid UTF-8, show as hex
                print(f"Binary data: {data.hex()}")
            
            print("-" * 50)
            
    except KeyboardInterrupt:
        print("\nShutting down UDP receiver...")
    except PermissionError:
        print(f"Error: Permission denied. Port {port} may require root privileges.")
        print("Try running with sudo or use a port number > 1024")
    except OSError as e:
        print(f"Error: {e}")
        print(f"Port {port} may already be in use.")
    finally:
        sock.close()

if __name__ == "__main__":
    # Allow port to be specified as command line argument
    port = 5009
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print("Invalid port number. Using default port 5000.")
    
    udp_receiver(port)
