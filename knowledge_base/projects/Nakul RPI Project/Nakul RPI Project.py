import RPi.GPIO as GPIO
from picamera import PiCamera

# Define a function to list sensor modules
def list_sensor_modules():
    sensor_modules = [
        "DHT22",
        "HC-SR04",
        "LED"
    ]
    return sensor_modules

# Define a function to list cameras
def list_cameras():
    cameras = [
        "Raspberry Pi Camera Module"
    ]
    return cameras

# Define a function to list DSPs
def list_dsps():
    dsps = [
        "Texas Instruments DSP"
    ]
    return dsps

# Define a function to list SoMs
def list_soms():
    soms = [
        "System-on-Module (SoM)"
    ]
    return soms

# Define a function to list FPGAs
def list_fpgas():
    fpgas = [
        "Xilinx FPGA"
    ]
    return fpgas

# Define a function to list CPNX modules
def list_cpnx_modules():
    cpnx_modules = [
        "CPNX modules"
    ]
    return cpnx_modules

# Define a function to list CM5
def list_cm5():
    cm5 = [
        "CM5"
    ]
    return cm5

# Main function
def main():
    print("Sensor Modules:")
    for sensor in list_sensor_modules():
        print(sensor)

    print("\nCameras:")
    for camera in list_cameras():
        print(camera)

    print("\nDSPs:")
    for dsp in list_dsps():
        print(dsp)

    print("\nSoMs:")
    for som in list_soms():
        print(som)

    print("\nFPGAs:")
    for fpga in list_fpgas():
        print(fpga)

    print("\nCPNX modules:")
    for cpnx in list_cpnx_modules():
        print(cpnx)

    print("\nCM5:")
    for cm in list_cm5():
        print(cm)

if __name__ == "__main__":
    main()