import time
import meshtastic
import meshtastic.serial_interface
from pubsub import pub

PORT = "xxxx"  # Change this to your receiver ESP32 port

def on_receive(packet, interface):
    decoded = packet.get("decoded", {})
    text = decoded.get("text")
    if text:
        sender = packet.get("fromId", "unknown")
        print(f"Received from {sender}: {text}", flush=True)
    
    else:
        print("Received non-text packet:", packet, flush=True)

interface = meshtastic.serial_interface.SerialInterface(devPath=PORT)

pub.subscribe(on_receive, "meshtastic.receive")

print("Listening for Meshtastic messages...")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    interface.close()
    print("Stopped.")