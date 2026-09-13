# C2000 SPI, ADC & PWM
Embedded communication and signal-generation project developed on the **TI TMS320F28379D (C2000)**.
The system samples 14 ADC channels every 100 µs, builds an 18-word data packet, and supports SPI transmission using a double-buffer structure.
SPI communication was validated using:
- SPI-A internal loopback
- SPI-A → SPI-B communication
- `HELLO` transmit/receive comparison
- Timeout and pass/fail checks
The project also includes **7 configurable ePWM outputs** on GPIO0–GPIO6.  
Their duty cycles can be changed at runtime through debug variables (`g_pwm1_duty` to `g_pwm7_duty`) from 0% to 100%.
The PWM outputs were also verified using an oscilloscope.
**MCU:** TI TMS320F28379D  
**IDE:** Code Composer Studio  
**Language:** C
