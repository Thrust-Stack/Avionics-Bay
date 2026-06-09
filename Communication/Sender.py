import time
import meshtastic
import meshtastic.serial_interface

# Windows example: "COM3"
# Linux example: "/dev/ttyUSB0" or "/dev/ttyACM0"
# macOS example: "/dev/cu.usbmodemXXXX"
PORT = "COM8"

interface = meshtastic.serial_interface.SerialInterface(devPath=PORT)

message = "Hello my name is Anderdingus .... OHHH SHITTINGS!!!"
interface.sendText(message)

print("Sent:", message)

time.sleep(2)
interface.close()