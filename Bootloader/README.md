# C2000 Custom Flash Bootloader
Custom flash bootloader developed for the **TI TMS320F28379D (C2000 CPU1)** with SPI communication to an STM32H7.
The bootloader starts from `0x080000`, receives 14 metadata words from the STM32H7 through SPI, validates and stores the metadata in Flash using the F021 Flash API, checks the application located at `0x0A0000`, and jumps to it when valid.
## Features
- SPI-A configured as Slave, Mode 0, 16-bit
- 14-word application metadata reception
- Metadata validation before Flash programming
- F021 Flash erase, blank-check, program, and verify
- Application validation before execution
- Direct jump to application at `0x0A0000`
- GPIO31 and debug-state variables for status/error monitoring
- Flash programming functions executed from RAM
## Memory Configuration
The linker configuration was also modified to separate the Bootloader, Application, and Metadata regions:
- Bootloader start: `0x080000`
- Application start: `0x0A0000`
- Metadata address: `0x0BE000`
These linker changes ensure that the Bootloader and Application use separate Flash regions and prevent the application area from being overwritten during Bootloader programming.
## Current Scope
This version handles SPI metadata reception, Flash metadata programming, application validation, and the final jump to the application.
The application firmware is currently programmed separately at `0x0A0000`.
Full firmware reception, application Flash programming, and CRC-based validation are intended for the next development stage.
**MCU:** TI TMS320F28379D  
**IDE:** Code Composer Studio  
**Language:** C
