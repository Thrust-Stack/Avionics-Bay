import time
import meshtastic
import meshtastic.serial_interface

# Windows example: "COM3"
# Linux example: "/dev/ttyUSB0" or "/dev/ttyACM0"
# macOS example: "/dev/cu.usbmodemXXXX"
PORT = "COM8"

interface = meshtastic.serial_interface.SerialInterface(devPath=PORT)

messages = [
    "Hello my name is Anderdingus!!!!!!!!!!",
    "OHHH SHITTINGS!!!",
    "Im boutta BUSSTTTTTT",
    "I Finished!!!",
]

for message in messages:
    interface.sendText(message)
    print("Sent:", message)
    time.sleep(8)

time.sleep(1)
interface.close()