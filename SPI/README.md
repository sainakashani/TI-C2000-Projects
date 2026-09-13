# C2000 SPI Communication
SPI and ADC communication project developed on the **TI TMS320F28379D (C2000)**.
The system samples 14 ADC channels every 100 µs and prepares an 18-word packet containing a header, sequence number, ADC samples, XOR checksum, and end marker using a double-buffer structure.
## SPI Validation
- SPI-A configured as Master
- SPI-B configured as Slave
- Internal SPI-A loopback test available
- SPI-A → SPI-B communication tested using `HELLO`
- Transmitted and received words are compared with timeout detection
- GPIO31 indicates a successful SPI-A → SPI-B test
The code also includes configurable ePWM outputs for oscilloscope testing and a packet transmission function for external SPI communication.
**MCU:** TI TMS320F28379D  
**IDE:** Code Composer Studio  
**Language:** C
