"""
Super Simple Wrist Controller API
Connect once, control continuously!
"""

import serial
import time

class Wrist:
    """Simple wrist controller - connect once, use many times"""
    
    def __init__(self, port):
        """
        Initialize and connect to wrist controller
        
        Args:
            port: Serial port (e.g., 'COM3' on Windows, '/dev/ttyACM0' on Linux)
        """
        self.port = port
        self.ser = None
        self.connect()
    
    def connect(self):
        """Connect to the wrist controller"""
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.1)
            time.sleep(0.5)  # Let it initialize
            print(f"Connected to wrist on {self.port}")
        except Exception as e:
            print(f"Connection failed: {e}")
            self.ser = None
    
    def control(self, pitch=0, roll=0, ee=0):
        """
        Control the wrist
        
        Args:
            pitch: Pitch PWM percentage (-100 to 100)
            roll: Roll PWM percentage (-100 to 100)  
            ee: End effector state (0=off, 1=close, 2=open, 3=hold)
        """
        if not self.ser:
            print("Not connected!")
            return
            
        try:
            # Clamp values to safe ranges
            pitch = max(-100, min(100, pitch))
            roll = max(-100, min(100, roll))
            ee = max(0, min(3, ee))
            
            # Send command
            command = f"{pitch},{roll},{ee}\n"
            self.ser.write(command.encode())
            self.ser.flush()
            
        except Exception as e:
            print(f"Error sending command: {e}")
    
    def stop(self):
        """Stop all movement"""
        self.control(0, 0, 0)
    
    def move(self, pitch, roll):
        """Move pitch and roll only"""
        self.control(pitch, roll, 0)
    
    def grab(self):
        """Close end effector"""
        self.control(0, 0, 1)
    
    def release(self):
        """Open end effector"""
        self.control(0, 0, 2)
    
    def close(self):
        """Close connection"""
        if self.ser:
            self.ser.close()
            print("Disconnected")
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()


# Example usage
if __name__ == "__main__":
    PORT = 'COM12'  # Change this to your port
    
    print("Simple Wrist Control Demo")
    
    # Method 1: Simple usage
    wrist = Wrist(PORT)
    
    wrist.stop()                    # Stop everything
    time.sleep(0.5)
    
    wrist.move(-10, -20)
    time.sleep(2)
    
    # wrist.move(10, 40)             # Pitch forward 50%, roll left 30%
    # time.sleep(1)
    
    # wrist.control(-10, 10, 0)       # Pitch back, roll right, end effector state 2
    # time.sleep(1)
    
    # wrist.grab()                    # Close end effector
    # time.sleep(3)

    # wrist.control(0,0,3)
    # time.sleep(3)
    
    # wrist.release()                 # Open end effector
    # time.sleep(3)
    
    wrist.stop()                    # Stop everything
    wrist.close()                   # Disconnect
    
    print("\n" + "="*50)
    
    # # Method 2: Context manager (auto-disconnect)
    # print("Using context manager:")
    # with Wrist(PORT) as w:
    #     w.move(30, 45)
    #     time.sleep(1)
    #     w.grab()
    #     time.sleep(1)
    #     w.stop()
    
    # print("Demo complete!")
