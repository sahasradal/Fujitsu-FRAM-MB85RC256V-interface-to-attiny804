# Fujitsu-FRAM-MB85RC256V-interface-to-attiny804
simple attempt to interface Fujitsu MB85RC256V FRAM to attiny804, writes 6 bytes and reads back from FRAM and transmits over UART
processor ATTINY804
memory chip - Fujitsu-FRAM-MB85RC256V (I2C)
attiny 804 pins used
power
ground
; PB0  = I2C clock (SCK) add 4k7 pullup resistor
; PB1  = I2C data  (SDA) add 4k7 pullup resistor
; PA1  = UART TX
; PA2  = UART RX

writes 6 bytes into the FRAM startin at address 0x0000 using I2C bus
after 2 seconds delay reads the FRAM and stores in the chips RAM and transmits over UART in a loop, this was a test attempt and was succesful
