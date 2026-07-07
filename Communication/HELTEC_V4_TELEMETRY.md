# Heltec V4 point-to-point telemetry

- Flash `HeltecV4TelemetryTransmitter/HeltecV4TelemetryTransmitter.ino` to the avionics board.
- Flash `HeltecV4TelemetryReceiver/HeltecV4TelemetryReceiver.ino` to the ground board.
- Install **RadioLib** through Arduino Library Manager and select the exact Heltec WiFi LoRa 32 V4 board revision.
- Attach the correct LoRa antenna before transmitting. The V4 also has a separate 2.4 GHz connector; do not confuse them.
- Open both Serial monitors at 115200 baud.

Both sketches default to 915 MHz, SF7, 125 kHz bandwidth, coding rate 4/5,
an eight-symbol preamble, and a 500 ms interval. SF7/BW125 is a useful starting
point for a 2 Hz, 20-byte link: it gives low airtime while retaining LoRa's link
budget. If range is insufficient, first improve antenna placement, then try SF8
or SF9. Each SF step roughly doubles symbol time, so higher spreading factors
reduce capacity and increase collision/duty-cycle exposure.

At SF7/BW125, 2 Hz is conservative for this payload and substantially higher
rates can work on a quiet point-to-point bench link. In practice, terrain,
airframe shielding, antenna orientation, interference, regional duty-cycle and
power limits, and the exact V4 hardware revision dominate reliability. There is
no acknowledgement or retry, by design; the packet counter lets the ground
receiver report gaps. Test the complete installed radio/antenna system at the
required distance before flight.

`LORA_FREQUENCY_MHZ`, power, spreading factor, and legal airtime vary by region.
915 MHz is a US-oriented default, not a universal setting. Both sketches must
always use identical frequency, bandwidth, spreading factor, coding rate, sync
word, preamble, CRC, and packet layout.

## Two-computer packet-delivery test

This test matches packet IDs from both computers, so their clocks do not need
to be synchronized. The transmitter must be compiled with **USB CDC On Boot:
Enabled** for its successful-transmit lines to be visible over USB.

On both computers, copy the `Communication` folder and install the serial
dependency:

```powershell
py -m pip install -r Communication\requirements-link-test.txt
```

Connect the avionics transmitter to computer 1 and the ground receiver to
computer 2. Find each current port with `arduino-cli board list`. Start the
captures at roughly the same time; a few seconds of difference is acceptable.

Computer 1 (replace `COM7` if needed):

```powershell
py Communication\heltec_link_test.py tx --port COM7 --duration 120 --output tx_report.json
```

Computer 2 (replace `COM9` if needed):

```powershell
py Communication\heltec_link_test.py rx --port COM9 --duration 120 --output rx_report.json
```

Copy both JSON files onto either computer, then compare the overlapping packet
ID range:

```powershell
py Communication\heltec_link_test.py compare tx_report.json rx_report.json
```

The comparison prints packets sent successfully, packets received, missing
packet IDs, and delivery percentage. The receiver report also includes average
RSSI and SNR. For a useful range test, run for at least 120 seconds, keep the
ground antenna fixed, and record distance, antenna orientation, obstructions,
and LoRa settings alongside the reports.
