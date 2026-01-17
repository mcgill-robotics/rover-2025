import subprocess

def get_ACM_port(default_port: str = "0") -> str:
    """
    Returns the ACM port number required for running the arm and drive firmware nodes. 
    Uses bash commands through subprocess to obtain the (first) ACM port occurence.
    If not found, defaults to a default port number, which can be given as an argument.
    """
    try:
        result = subprocess.run(["sudo", "dmesg"], capture_output=True, text=True)
        result = subprocess.run(["grep", "TCP"], input=result.stdout, capture_output=True, text=True)
        result = subprocess.run(["tr", "-s", " "], input=result.stdout, capture_output=True, text=True)
        result = subprocess.run(["cut", "-d", " ", "-f6"], input=result.stdout, capture_output=True, text=True)
        result = subprocess.run(["grep", "-o", "-E", "[0-9]+"], input=result.stdout, capture_output=True, text=True)
        port = result.stdout.split("\n")[0]

        if not port:
            port = default_port
    except:
        port = default_port
    return port