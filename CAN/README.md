# C2000 CAN Communication
CAN communication project developed on the **TI TMS320F28379D (C2000)**.
The system samples 14 ADC channels every 100 µs, creates a 36-byte packet, and transmits it over CAN-A at 500 kbps.
Because Classical CAN supports up to 8 data bytes per frame, the packet is divided into 5 CAN frames (8 + 8 + 8 + 8 + 4 bytes).
## Features
- ADC sampling from ADCA, ADCB, and ADCC
- 36-byte packet with header, sequence number, checksum, and end marker
- Packet fragmentation into 5 CAN frames
- CAN-A communication at 500 kbps
- External loopback verification
- Byte-by-byte TX/RX comparison
- Timeout and debug counters
- Double buffering for packet handling
**MCU:** TI TMS320F28379D  
**IDE:** Code Composer Studio  
**Language:** C
