import time
import meshtastic
import meshtastic.serial_interface
from pubsub import pub

PORT = "COM4"  # Change this to your receiver ESP32 port

def on_receive(packet, interface):
    decoded = packet.get("decoded", {})
    text = decoded.get("text")
    if text:
        sender = packet.get("fromId", "unknown")
        print(f"Received from {sender}: {text}")
    else:
        print("Received packet:", packet)

pub.subscribe(on_receive, "meshtastic.receive.text")

interface = meshtastic.serial_interface.SerialInterface(devPath=PORT)

print("Listening for Meshtastic messages...")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    interface.close()
    print("Stopped.")