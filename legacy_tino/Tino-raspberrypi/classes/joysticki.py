from evdev import InputDevice, categorize, ecodes

gamepad = InputDevice('/dev/input/by-id/usb-Logitech_Logitech_Cordless_RumblePad_2-event-joystick')

class Controller:
    def loop(self):
        try:
            for event in gamepad.read_loop():
                if event.code == 0 and event.type == 0:
                    pass
                else:
                    print("---------------------------------------------------------------" + 
                          " | code:" + str(event.code) + 
                          " | type:" + str(event.type) + 
                          " | value:" + str(event.value))
                    
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            gamepad.ungrab()  # Rilascia il gamepad in caso di interruzione

if __name__ == "__main__":
    controller = Controller()
    controller.loop()