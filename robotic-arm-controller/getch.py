import sys
import os

if os.name == "nt":
    import msvcrt
else:
    import tty
    import termios

def clear():
    os.system('cls' if os.name == 'nt' else 'clear')

def getch():

    ch = ''
    if os.name == "nt":
        ch = msvcrt.getch()

        if ch == b'\x18':
            return "CTRL+X"

        if ch == b'\xe0' or ch == '\x00':
            ch2 = msvcrt.getch()
            if ch2 == b'H':
                return "UP_ARROW"
            elif ch2 == b'P':
                return "DOWN_ARROW"
            elif ch2 == b'M':
                return "RIGHT_ARROW"
            elif ch2 == b'K':
                return "LEFT_ARROW"
            else:
                return "UNKNOWN_ESCAPE_SEQUENCE"
            
        return ch.decode(errors='ignore')
    else:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if ch == '\x1b':  # Escape character
            ch += sys.stdin.read(2)  # Read the next two characters
            if ch == '\x1b[A':
                return "UP_ARROW"
            elif ch == '\x1b[B':
                return "DOWN_ARROW"
            elif ch == '\x1b[C':
                return "RIGHT_ARROW"
            elif ch == '\x1b[D':
                return "LEFT_ARROW"
            else:
                return "UNKNOWN_ESCAPE_SEQUENCE"  # Handle other escape sequences 

        return ch  # Return regular key press if not an escape sequence