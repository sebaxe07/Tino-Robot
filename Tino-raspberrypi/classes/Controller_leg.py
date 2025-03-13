import serial
from datetime import datetime
import multiprocessing
from evdev import InputDevice, categorize, ecodes
import time
import csv

SERIAL_RATE = 0.1
TOT_MSG = 2
serial_base_port = "/dev/ttyUSB0_LEG"
ser_base = serial.Serial(serial_base_port, 115200, timeout=1)

save_path = '/home/tino/Desktop/Tino/'

time.sleep(2)  # Attendi che la connessione seriale si stabilisca

gamepad = InputDevice('/dev/input/by-id/usb-Logitech_Logitech_Cordless_RumblePad_2-event-joystick')

_manager = multiprocessing.Manager()
_gamepadState = _manager.dict()

def mapRange(value, inMin, inMax, outMin, outMax):
    return outMin + (((value - inMin) / (inMax - inMin)) * (outMax - outMin))

def serial_writer(_gamepadState, write_queue):
    print("serial_writer started")
    prev_time = datetime.now()
    while True:
        if (datetime.now() - prev_time).total_seconds() < SERIAL_RATE:
            continue
        else:
            send_base_str = ("BF:%.2f" % _gamepadState["BF"] + "_BB:%.2f" % _gamepadState["BB"])
            # send_base_str = ("BF:%.2f" % _gamepadState["BF"])
            write_queue.put(send_base_str)
            prev_time = datetime.now()

# # Funzione per salvare i dati ricevuti
# def save_received_data(data):
#     with open(save_path + 'received_data.csv', mode='a', newline='') as file:
#         writer = csv.writer(file)
#         timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
#         writer.writerow([timestamp, data])

def serial_reader(read_queue):
    print("serial_reader started")
    while True:
        if ser_base.in_waiting > 0:
            try:
                line = ser_base.readline().decode('utf-8').rstrip()
                read_queue.put(line)
                # save_received_data(line)
            except UnicodeDecodeError:
                print("Decoding error occurred.")
            except Exception as e:
                print(f"Unexpected error: {e}")

def serial_comm(write_queue, read_queue):
    print("serial_comm started")
    while True:
        if not write_queue.empty():
            send_base_str = write_queue.get()
            for _ in range(TOT_MSG):
                ser_base.write((send_base_str + "\n").encode('utf-8'))

        if not read_queue.empty():
            line = read_queue.get()
            print("[ARDUINO]" + line)

class Controller:
    def loop(self):
        write_queue = multiprocessing.Queue()
        read_queue = multiprocessing.Queue()

        serial_writer_process = multiprocessing.Process(target=serial_writer, args=(_gamepadState, write_queue))
        serial_reader_process = multiprocessing.Process(target=serial_reader, args=(read_queue,))
        serial_comm_process = multiprocessing.Process(target=serial_comm, args=(write_queue, read_queue))

        serial_writer_process.start()
        serial_reader_process.start()
        serial_comm_process.start()

        gamepad.grab()

        _gamepadState["BF"] = 0
        _gamepadState["BB"] = 0

        try:
            for event in gamepad.read_loop():
                if event.code == 0 and event.type == 0:
                    pass
                else:
                    eventName = 0
                    if event.code == 2:
                        eventName = "BB"
                    elif event.code == 5:  #elif
                        eventName = "BF"

                    if event.code == 5:
                        if 120 <= event.value <= 130:
                            new_val = 0
                        else:
                            new_val = mapRange(event.value, 130, 255, 0, 0) if event.value > 130 else mapRange(event.value, 0, 120, 130, 90)
                        _gamepadState[eventName if eventName else event.code] = new_val

                    if event.code == 2:
                        if 110 <= event.value <= 140:
                            new_val = 0
                        else:
                            new_val = mapRange(event.value, 140, 255, 0, -1.1) if event.value > 140 else mapRange(event.value, 0, 110, 1.1, 0)
                        _gamepadState[eventName if eventName else event.code] = new_val

        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            gamepad.ungrab()  # Questo garantisce che il gamepad venga rilasciato correttamente anche in caso di interruzione

if __name__ == "__main__":
    controller = Controller()
    controller.loop()
