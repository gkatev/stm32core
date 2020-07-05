# stm32core

A simple core I developed for use with STM32F103 devices.  
Definetely still in alpha/beta, but in a well-working state IMO.

Includes:
* DFU bootloader with small mods for easier programming 
* Interrupt driven USB CDC ACM with integrated quick reset into DFU mode
* Interrupt driven UART library
* printf support for UART/CDC, courtesy of mini_printf
* Simple Arduino-like millis() counter

Longer Readme potentially coming soon!
