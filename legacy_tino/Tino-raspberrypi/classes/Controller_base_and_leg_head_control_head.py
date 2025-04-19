import serial
from datetime import datetime
import multiprocessing
from evdev import InputDevice, categorize, ecodes
import time
import csv

SERIAL_RATE = 0.04  # Intervallo tra i messaggi seriali
TOT_MSG = 3  # Numero di messaggi da inviare
serial_base_port = "/dev/ttyUSB0_BASE"  # Prima connessione seriale
serial_leg_port = "/dev/ttyUSB0_LEG"    # Seconda connessione seriale
serial_head_port = "/dev/ttyUSB0_HEAD"    # Terza connessione seriale

# Inizializza le due connessioni seriali
ser_base = serial.Serial(serial_base_port, 115200, timeout=1)
ser_leg = serial.Serial(serial_leg_port, 115200, timeout=1)
ser_head = serial.Serial(serial_head_port, 115200, timeout=1)


save_path = '/home/tino/Desktop/Tino/'

time.sleep(2)  # Attendi che le connessioni seriali si stabiliscano

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
            # Prepara il messaggio da inviare
            send_base_str = ("BF:%.2f" % _gamepadState["BF"] + "_BB:%.2f" % _gamepadState["BB"] + "_HP:%.2f" % _gamepadState["HP"] + "_HX:%.2f" % _gamepadState["HX"]+ "_HY:%.2f" % _gamepadState["HY"])
            write_queue.put(send_base_str)  # Inserisce il messaggio nella coda di invio
            prev_time = datetime.now()

def serial_reader(read_queue, ser):
    print("serial_reader started")
    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').rstrip()
                read_queue.put(line)  # Aggiungi il dato letto alla coda
            except UnicodeDecodeError:
                print("Decoding error occurred.")
            except Exception as e:
                print(f"Unexpected error: {e}")

def serial_comm(write_queue, read_queue, ser_base, ser_leg, ser_head):
    print("serial_comm started")
    while True:
        if not write_queue.empty():
            send_base_str = write_queue.get()  # Ottiene il messaggio dalla coda

            # Invia lo stesso messaggio a entrambi gli Arduini
            for _ in range(TOT_MSG):
                ser_base.write((send_base_str + "\n").encode('utf-8'))
                ser_leg.write((send_base_str + "\n").encode('utf-8'))
                ser_head.write((send_base_str + "\n").encode('utf-8'))

        # Leggi i dati ricevuti da entrambi gli Arduini (facoltativo)
        if not read_queue.empty():
            line = read_queue.get()
            print("[ARDUINO]" + line)

class Controller:
    def loop(self):
        write_queue = multiprocessing.Queue()  # Coda per invio
        read_queue = multiprocessing.Queue()   # Coda per ricezione

        # Avvia i processi di scrittura e lettura seriale
        serial_writer_process = multiprocessing.Process(target=serial_writer, args=(_gamepadState, write_queue))
        serial_reader_base_process = multiprocessing.Process(target=serial_reader, args=(read_queue, ser_base))
        serial_reader_leg_process = multiprocessing.Process(target=serial_reader, args=(read_queue, ser_leg))
        serial_reader_head_process = multiprocessing.Process(target=serial_reader, args=(read_queue, ser_head))
        serial_comm_process = multiprocessing.Process(target=serial_comm, args=(write_queue, read_queue, ser_base, ser_leg,ser_head))

        serial_writer_process.start()
        serial_reader_base_process.start()
        serial_reader_leg_process.start()
        serial_reader_head_process.start()
        serial_comm_process.start()

        gamepad.grab()

        _gamepadState["BF"] = 0
        _gamepadState["BB"] = 0
        _gamepadState["HP"] = 0
        _gamepadState["HX"] = 511
        _gamepadState["HY"] = 511

        try:
            for event in gamepad.read_loop():
                if event.code == 0 and event.type == 0:
                    pass
                else:
                    print("---------------------------------------------------------------" + " | code:" + str(event.code) + " | type:" + str(event.type) + " | value:" + str(event.value))
                    eventName = 0
                    if event.code == 2:
                        eventName = "BB"
                    elif event.code == 5:  #elif
                        eventName = "BF"
                    elif event.code == 1:
                        eventName = "HX"
                    elif (event.code == 0 and event.type == 3):
                        eventName = "HY"
                    elif event.code == 17:
                        eventName = "HP"

                    if event.code == 5:
                        if 120 <= event.value <= 130:
                            new_val = 0
                        else:
                            new_val = mapRange(event.value, 130, 255, 0, 0) if event.value > 130 else mapRange(event.value, 0, 120, 25, 0)
                        _gamepadState[eventName if eventName else event.code] = new_val

                    if event.code == 2:
                        if 110 <= event.value <= 140:
                            new_val = 0
                        else:
                            new_val = mapRange(event.value, 140, 255, 0, -1.1) if event.value > 140 else mapRange(event.value, 0, 110, 1.1, 0)
                        _gamepadState[eventName if eventName else event.code] = new_val
                        
                        
                    if(event.code == 0 and event.type == 3):
                        if 120 <= event.value <= 130:
                            new_val = 511
                        else:
                            if(event.value < 120):
                                new_val = mapRange(event.value, 130, 255, 512, 1024)
                            else:
                                new_val = mapRange(event.value, 0, 120, 0, 511)
                        _gamepadState[eventName if eventName else event.code] = new_val  
                    
                    if event.code == 1:
                        if 120 <= event.value <= 130:
                            new_val = 511
                        else:
                            if event.value < 120:
                                new_val = mapRange(event.value, 130, 255, 512, 1024)
                            else:
                                new_val = mapRange(event.value, 0, 120, 0, 511)
                        _gamepadState[eventName if eventName else event.code] = new_val
                        
                    if event.code == 17:
                        if event.value == 0:
                            new_val = 0
                        if event.value < 0:
                            new_val = -335
                        elif event.value > 0:
                            new_val = 335
                        _gamepadState[eventName if eventName else event.code] = new_val  
                        # print (_gamepadState)

        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            gamepad.ungrab()  # Rilascia il gamepad in caso di interruzione

if __name__ == "__main__":
    controller = Controller()
    controller.loop()