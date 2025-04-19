import os
import multiprocessing
import controller.Controller_camera_working_2 as Controller
from stream.Stream import app



# ______________________________________________________________________________________________GLOBALS

# directory of the file. It's the same dicrectory of the RESTART.SH file
abs_path = os.path.dirname(os.path.abspath(__file__))
restart_file_name = "restart.sh"
path_to_restart = "./" + restart_file_name  # abs_path + "/restart.sh"

CONTROLLER_ENABLED = 1

# Event for signaling termination
terminate_event = multiprocessing.Event()

# ______________________________________________________________________________________________ FUNCTIONS

def start_streaming():
    app.run(host='0.0.0.0', port=5000, threaded=True)
    
def main():
    if CONTROLLER_ENABLED:
        controller_process = Controller.Controller()
        
        # Start the streaming process
        stream_process = multiprocessing.Process(target=start_streaming)
        stream_process.start()

        try:
            # Run the robot control loop, checking for termination event
            controller_process.loop(terminate_event)
        except KeyboardInterrupt:
            print("Received keyboard interrupt, shutting down.")
        finally:
            # Signal termination and join processes
            terminate_event.set()  # Signal to terminate streaming process
            stream_process.join()  # Wait for the streaming process to finish
         
    
if __name__ == "__main__":
    main()
