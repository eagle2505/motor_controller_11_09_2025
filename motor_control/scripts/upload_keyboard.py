#!/usr/bin/env python3

import subprocess
import os
import sys

def run_command(command):
    try:
        result = subprocess.run(command, shell=True, check=True, text=True, capture_output=True)
        print(result.stdout)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")
        print(f"Error output: {e.stderr}")
        return False

def check_pid_library():
    # Check if PID library is installed
    check_cmd = "arduino-cli lib list | grep -q 'PID'"
    try:
        subprocess.run(check_cmd, shell=True, check=True)
        return True
    except subprocess.CalledProcessError:
        return False

def install_pid_library():
    print("PID library not found. Installing...")
    install_cmd = "arduino-cli lib install 'PID'"
    if not run_command(install_cmd):
        print("Failed to install PID library!")
        return False
    print("PID library installed successfully!")
    return True

def main():
    # Get the absolute path to the sketch
    sketch_path = "/home/roamio42/Desktop/mariam_branch/motor_control/arduino_scripts/keyboard_control"
    
    # Check if the sketch exists
    if not os.path.exists(sketch_path):
        print(f"Error: Could not find sketch at {sketch_path}")
        sys.exit(1)

    # Check and install PID library if needed
    if not check_pid_library():
        if not install_pid_library():
            print("Failed to install required library. Exiting...")
            sys.exit(1)

    # Compile command
    compile_cmd = f"arduino-cli compile --fqbn arduino:avr:mega {sketch_path}"
    print("Compiling sketch...")
    if not run_command(compile_cmd):
        print("Compilation failed!")
        sys.exit(1)
    
    # Upload command
    upload_cmd = f"arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega {sketch_path}"
    print("\nUploading sketch...")
    if not run_command(upload_cmd):
        print("Upload failed!")
        sys.exit(1)
    
    print("\nUpload completed successfully!")

if __name__ == "__main__":
    main() 