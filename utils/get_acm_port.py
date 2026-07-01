import subprocess
from enum import Enum

"""
enum of subsystems and their associated serial numbers
use "udevadm info -q all -a -n /dev/ttyACMx" to get this info about the device in serial port x
serial number should be in a line that contains "ATTRS{serial}"
"""
class Subsystem(Enum):
    DRIVE = "28003E001950453055373020"
    ARM = "TODO: obtain this"
    GPS = "2087338C3630"

def get_ACM_port(subsystem: Subsystem = Subsystem.DRIVE) -> str:
    """
    Returns the ACM port number required for running the associated robot subsystem. 
    Uses bash commands through subprocess to obtain the active ACM port with a matching serial number
    If not found successfully, it returns '-1'
    """
            
    try:
        # get log history
        result = subprocess.run(["sudo", "dmesg"], capture_output=True, text=True) 
        # search for ACM port mentions
        result = subprocess.run(["grep", "ttyACM"], input=result.stdout, capture_output=True, text=True)
        # squash groups of multiple ' ' characters into one
        result = subprocess.run(["tr", "-s", " "], input=result.stdout, capture_output=True, text=True)
        # delimiting by spaces, take the 5th field (ACM port name)
        result = subprocess.run(["cut", "-d", " ", "-f5"], input=result.stdout, capture_output=True, text=True)
        # find all numbers contained in these strings
        result = subprocess.run(["grep", "-o", "-E", "[0-9]+"], input=result.stdout, capture_output=True, text=True)
        # convert to an array of active ACM ports
        ports = result.stdout.split("\n")
        
        # if array is empty return
        if not ports[0]:
            print("no active ACM ports found")
            return -1
        
        # for each active ACM port
        for port in ports:
            name = "/dev/ttyACM" + port
            # get its info
            result = subprocess.run(["udevadm", "info", "-q", "all", "-a", "-n", name], capture_output=True, text=True)
            # look for the line containing the serial number of the connected device
            result = subprocess.run(["grep", f"ATTRS{{serial}}"], input=result.stdout, capture_output=True, text=True)
            # delimiting by double quotation makrs, take the 2nd field (serial number)
            result = subprocess.run(["cut", "-d", '"', "-f2"], input=result.stdout, capture_output=True, text=True)
            # convert to an array of serial numbers
            ids = result.stdout.split("\n")
            
            # for each serial number
            for id in ids:
                # if it matches the desired subsystem return the port
                if(id == subsystem.value):
                    print(f"{subsystem.name} connected at ttyACM{port}")
                    return port
    except:
        print("there was an error detecting ACM ports")
        return -1
    
    print(f"{subsystem.name} not detected on any active ports")
    return -1