import serial
from datetime import datetime
import multiprocessing
from evdev import InputDevice
import time

# Constants for serial communication and gamepad
SERIAL_RATE = 0.04  # Interval between serial messages
TOT_MSG = 3  # Number of messages to send
serial_base_port = "/dev/ttyUSB0_BASE"
serial_leg_port = "/dev/ttyUSB0_LEG"
serial_head_port = "/dev/ttyUSB0_HEAD"

# Initialize serial connections
ser_base = serial.Serial(serial_base_port, 115200, timeout=1)
ser_leg = serial.Serial(serial_leg_port, 115200, timeout=1)
ser_head = serial.Serial(serial_head_port, 115200, timeout=1)

time.sleep(2)  # Wait for serial connections to stabilize

gamepad = InputDevice('/dev/input/by-id/usb-Logitech_Logitech_Cordless_RumblePad_2-event-joystick')

_manager = multiprocessing.Manager()
_gamepadState = _manager.dict()

def mapRange(value, inMin, inMax, outMin, outMax):
    return outMin + (((value - inMin) / (inMax - inMin)) * (outMax - outMin))

def serial_writer(_gamepadState, write_queue, terminate_event):
    print("serial_writer started")
    prev_time = datetime.now()
    while not terminate_event.is_set():
        if (datetime.now() - prev_time).total_seconds() < SERIAL_RATE:
            continue
        else:
            send_base_str = ("BF:%.2f" % _gamepadState["BF"] +
                             "_BB:%.2f" % _gamepadState["BB"] +
                             "_HP:%.2f" % _gamepadState["HP"] +
                             "_HX:%.2f" % _gamepadState["HX"] +
                             "_HY:%.2f" % _gamepadState["HY"])
            write_queue.put(send_base_str)  # Put message in the queue
            prev_time = datetime.now()

def serial_reader(read_queue, ser, terminate_event):
    print("serial_reader started")
    while not terminate_event.is_set():
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').rstrip()
                read_queue.put(line)  # Add read data to queue
            except UnicodeDecodeError:
                print("Decoding error occurred.")
            except Exception as e:
                print(f"Unexpected error: {e}")

def serial_comm(write_queue, read_queue, ser_base, ser_leg, ser_head, terminate_event):
    print("serial_comm started")
    while not terminate_event.is_set():
        if not write_queue.empty():
            send_base_str = write_queue.get()  # Get message from queue

            # Send message to all Arduinos
            for _ in range(TOT_MSG):
                ser_base.write((send_base_str + "\n").encode('utf-8'))
                ser_leg.write((send_base_str + "\n").encode('utf-8'))
                ser_head.write((send_base_str + "\n").encode('utf-8'))

        if not read_queue.empty():
            line = read_queue.get()
            print("[ARDUINO] " + line)

class Controller:
    def loop(self, terminate_event):
        write_queue = multiprocessing.Queue()  # Queue for sending data
        read_queue = multiprocessing.Queue()   # Queue for receiving data

        # Start processes for writing and reading serial data
        serial_writer_process = multiprocessing.Process(target=serial_writer, args=(_gamepadState, write_queue, terminate_event))
        serial_reader_base_process = multiprocessing.Process(target=serial_reader, args=(read_queue, ser_base, terminate_event))
        serial_reader_leg_process = multiprocessing.Process(target=serial_reader, args=(read_queue, ser_leg, terminate_event))
        serial_reader_head_process = multiprocessing.Process(target=serial_reader, args=(read_queue, ser_head, terminate_event))
        serial_comm_process = multiprocessing.Process(target=serial_comm, args=(write_queue, read_queue, ser_base, ser_leg, ser_head, terminate_event))

        # Start all processes
        serial_writer_process.start()
        serial_reader_base_process.start()
        serial_reader_leg_process.start()
        serial_reader_head_process.start()
        serial_comm_process.start()

        gamepad.grab()

        try:
            while not terminate_event.is_set():
                _gamepadState["BF"] = 0
                _gamepadState["BB"] = 0
                _gamepadState["HP"] = 0
                _gamepadState["HX"] = 511
                _gamepadState["HY"] = 511

                for event in gamepad.read_loop():
                    if event.code == 0 and event.type == 0:
                        continue
                    
                    eventName = None
                    
                    if event.code == 309:
                        eventName = "BB"  # Destra
                    elif event.code == 308:
                        eventName = "BB"  # Sinistra
                    elif event.code == 5:
                        eventName = "BF"
                    elif event.code == 1:
                        eventName = "HX"
                    elif (event.code == 0 and event.type == 3):
                        eventName = "HY"
                    elif event.code == 17:
                        eventName = "HP"

                    # Update gamepad state based on events
                    if eventName is not None:
                        new_val = self.process_gamepad_event(event)
                        _gamepadState[eventName] = new_val

        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            gamepad.ungrab()  # Release the gamepad on exit

    def process_gamepad_event(self, event):
        """Process individual gamepad events and return the new value."""
        if event.code == 5:  # BF button
            if 120 <= event.value <= 130:
                return 0
            else:
                return mapRange(event.value, 130, 255, 0, 0) if event.value > 130 else mapRange(event.value, 0, 120, 15, 0)
        
        elif event.code == 309:  # BB button destra
            if event.value == 1:
                return -1.1 
            else:
                return 0

        elif event.code == 308:  # BB button sinistra
            if event.value == 1:
                return 1.1  
            else :
                return 0  #           

        elif (event.code == 0 and event.type == 3):  
            if 120 <= event.value <= 130:
                return 511
            else:
                return mapRange(event.value, (130 if event.value <120 else (0)), (255 if event.value <120 else (120)), (512 if event.value <120 else (0)), (1024 if event.value <120 else (511)))

        elif event.code == 1:  
            if 120 <= event.value <= 130:
                return 511
            else:
                return mapRange(event.value,(130 if event.value <120 else (0)),(255 if event.value <120 else (120)),(512 if event.value <120 else (0)),(1024 if event.value <120 else (511)))

        elif event.code ==17:  
            return -335 if event.value <0 else (335 if event.value >0 else(0))

if __name__ == "__main__":
    controller = Controller()
    
    # Create a termination event for graceful shutdown.
    terminate_event = multiprocessing.Event()

    try:
        controller.loop(terminate_event)
    except KeyboardInterrupt:
        print("Received keyboard interrupt. Terminating...")
    finally:
        terminate_event.set()   # Signal termination to all processes.