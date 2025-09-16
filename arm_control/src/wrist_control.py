"""
Real-time Wrist Control with Arrow Keys
Works on Windows and Linux
"""

import sys
import time
import threading
from simple_wrist_api import Wrist

# Cross-platform keyboard input
try:
    import msvcrt  # Windows
    WINDOWS = True
except ImportError:
    import tty, termios  # Linux/Mac
    WINDOWS = False

class KeyboardController:
    def __init__(self, port):
        self.wrist = Wrist(port)
        self.pitch = 0
        self.roll = 0
        self.ee = 0
        self.running = True
        
        # Fixed speeds
        self.pitch_speed = 30
        self.roll_speed = 100
        
        # Start command sending thread
        self.command_thread = threading.Thread(target=self.send_commands, daemon=True)
        self.command_thread.start()
    
    def send_commands(self):
        """Continuously send commands to the wrist controller"""
        while self.running:
            try:
                self.wrist.control(self.pitch, self.roll, self.ee)
                time.sleep(0.05)  # Send at 20Hz
            except:
                pass
    
    def get_key_windows(self):
        """Get key press on Windows"""
        if msvcrt.kbhit():
            key = msvcrt.getch()
            # Handle arrow keys (they send 2 bytes)
            if key == b'\xe0':  # Special key prefix
                key = msvcrt.getch()
                return {
                    b'H': 'up',      # Up arrow
                    b'P': 'down',    # Down arrow  
                    b'K': 'left',    # Left arrow
                    b'M': 'right'    # Right arrow
                }.get(key, None)
            elif key == b'\x1b':  # ESC
                return 'esc'
            elif key.lower() == b'o':
                return 'open'
            elif key.lower() == b'c':
                return 'close'
            elif key.lower() == b's':
                return 'stop'
        return None
    
    def get_key_linux(self):
        """Get key press on Linux"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            
            if ch == '\x1b':  # ESC sequence
                ch2 = sys.stdin.read(1)
                if ch2 == '[':
                    ch3 = sys.stdin.read(1)
                    return {
                        'A': 'up',
                        'B': 'down', 
                        'C': 'right',
                        'D': 'left'
                    }.get(ch3, None)
                else:
                    return 'esc'
            elif ch.lower() == 'o':
                return 'open'
            elif ch.lower() == 'c':
                return 'close'
            elif ch.lower() == 's':
                return 'stop'
            elif ch == '\x03':  # Ctrl+C
                return 'esc'
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return None
    
    def get_key(self):
        """Cross-platform key getter"""
        if WINDOWS:
            return self.get_key_windows()
        else:
            return self.get_key_linux()
    
    def print_instructions(self):
        """Print control instructions"""
        print("\n" + "="*50)
        print("WRIST CONTROL - Real-time Arrow Key Control")
        print("="*50)
        print("Controls:")
        print("  ↑ ↓     - Pitch (±10%)")
        print("  ← →     - Roll (±20%)")
        print("  O       - Open end effector")
        print("  C       - Close end effector") 
        print("  S       - Stop all movement")
        print("  ESC     - Exit")
        print("="*50)
        print("Ready! Use keys to control (no feedback shown)...")
        print()
    
    def run(self):
        """Main control loop"""
        self.print_instructions()
        
        # Set up non-blocking input for Linux
        if not WINDOWS:
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            tty.setraw(sys.stdin.fileno())
        
        try:
            while self.running:
                key = self.get_key()
                
                if key == 'up':
                    self.pitch = self.pitch_speed
                elif key == 'down':
                    self.pitch = -self.pitch_speed
                elif key == 'left':
                    self.roll = -self.roll_speed
                elif key == 'right':
                    self.roll = self.roll_speed
                elif key == 'open':
                    self.ee = 2  # Open state
                elif key == 'close':
                    self.ee = 1  # Close state
                elif key == 'stop':
                    self.pitch = 0
                    self.roll = 0
                    self.ee = 0
                elif key == 'esc':
                    break
                
                # Small delay to prevent CPU spinning
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            pass
        finally:
            # Cleanup
            if not WINDOWS:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            
            self.running = False
            self.wrist.stop()
            self.wrist.close()
            print("\nWrist control stopped.")

def main():
    if len(sys.argv) != 2:
        print("Usage: python wrist_control_keys.py <COM_PORT>")
        print("Example: python wrist_control_keys.py COM3")
        print("Example: python wrist_control_keys.py /dev/ttyACM0")
        sys.exit(1)
    
    port = sys.argv[1]
    
    try:
        controller = KeyboardController(port)
        controller.run()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
