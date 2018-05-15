Arduino_AIS - A simple DIY AIS Receiver port of dAISy
=================================

dAISy (do AIS yourself) is a simple AIS receiver based on Silicon Labs [Si4362 EZRadioPro](http://www.silabs.com/products/wireless/ezradiopro/pages/si4362.aspx) receiver, Texas Instruments [MSP430G2553](http://www.ti.com/product/msp430g2553) MCU and [MSP-EXP430G2](http://www.ti.com/tool/msp-exp430g2) v1.5 LaunchPad. The software was developed with Code Composer Studio 5.5.

AIS, short for "Automatic Identification System", is a tracking system for ships. More on [Wikipedia](http://en.wikipedia.org/wiki/Automatic_Identification_System). There are many websites dedicated to tracking ships based on this system, like for example [MarineTraffic](http://www.marinetraffic.com/).

dAISy features:
- can be used with arduino Nano and an off the shelf si446x module
- integrated radio, no need for external radio with discriminator tap
- hopping between channel A (161.975 MHz) and B (162.025 MHz)
- receives, decodes and validates packets according to ITU-R M.1371-4 (NRZI decoding, bit-destuffing, CRC validation) 
- wraps valid packets into NMEA 0183 sentences (AIVDM)
- sends NMEA sentences to PC via serial (9600 8N1)

The output of dAISy can be processed and visualized by mapping and navigation programs like [OpenCPN](http://opencpn.org).

All content of this project is published under CC BY-NC-SA - [Creative Commons Attribution-NonCommercial-ShareAlike](http://creativecommons.org/licenses/by-nc-sa/4.0/). 
if using an arduino nano 
gnd on radio to gnd
gpio2 on radio to pin d2
irq on radio to pin d3
nsel on radio to pin d4
gpio1 on radio to pin d5
gpio3 on radio to pin d6
sdn on radio to pin d8 
mosi on radio to pin d11
miso on radio to pin d12
clock on radio to pin d13 

When buying the si446x module make sure it has a 30mhz crystal rather than 26mhz and that it has connectors for all the GPIO Pins you will have better success with the ones that don't have an antenna switch.
you can directly connect on a 3.3v Arduino or use 1k limiting resistors D4 D8 D11 , D13  
