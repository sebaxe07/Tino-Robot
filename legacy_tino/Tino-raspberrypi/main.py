import os
import classes.Controller_head as Controller

# ______________________________________________________________________________________________GLOBALS

# directory of the file. It's the same dicrectory of the RESTART.SH file
abs_path = os.path.dirname(os.path.abspath(__file__))
restart_file_name = "restart.sh"
path_to_restart = "./" + restart_file_name  # abs_path + "/restart.sh"

CONTROLLER_ENABLED = 1


# ______________________________________________________________________________________________ MAIN

def main():
    if CONTROLLER_ENABLED:
        controller_process = Controller.Controller()
        controller_process.loop()


if __name__ == "__main__":
    main()
