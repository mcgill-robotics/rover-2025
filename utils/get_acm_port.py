import subprocess
from enum import Enum

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
        result = subprocess.run(["sudo", "dmesg"], capture_output=True, text=True)
        result = subprocess.run(["grep", "ttyACM"], input=result.stdout, capture_output=True, text=True)
        result = subprocess.run(["tr", "-s", " "], input=result.stdout, capture_output=True, text=True)
        result = subprocess.run(["cut", "-d", " ", "-f5"], input=result.stdout, capture_output=True, text=True)
        result = subprocess.run(["grep", "-o", "-E", "[0-9]+"], input=result.stdout, capture_output=True, text=True)
        ports = result.stdout.split("\n")
        
        if not ports[0]:
            print("no active ACM ports found")
            return -1
        
        for port in ports:
            name = "/dev/ttyACM" + port
            result = subprocess.run(["udevadm", "info", "-q", "all", "-a", "-n", name], capture_output=True, text=True)
            result = subprocess.run(["grep", "ATTRS{{serial}}"], input=result.stdout, capture_output=True, text=True)
            result = subprocess.run(["cut", "-d", '"', "-f2"], input=result.stdout, capture_output=True, text=True)
            ids = result.stdout.split("\n")
            for id in ids:
                if(id == subsystem.value):
                    print("{subsystem.name} connected at ttyACM{port}")
                    return port
    except:
        print("there was an error detecting ACM ports")
        return -1
    print("{subsystem.name} not detected on any active ports")
    return -1