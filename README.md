# IthoEcoFanRFT
Cloned from supersjimmie who cloned from Klusjesman, work in progress to modify for Raspberry Pi - DO NOT USE YET, UNFINSIHED

Will work with a 868MHz CC1101 module.
The CC1150 may also work, except for receiving (which is not required for controlling an Itho EcoFan).
A 433MHz CC1101/CC1150 might also work, because it has the same chip. But a 433MHz CC11xx board has a different RF filter, causing a lot less transmission power (and reception).

Requires wiringPi!

```
Connections between the CC1101 and the ESP8266 or Arduino:
CC11xx pins    ESP pins Arduino pins  RPi pins  Description
*  1 - VCC        VCC      VCC         VCC        3v3
*  2 - GND        GND      GND         GND        Ground
*  3 - MOSI       13=D7    Pin 11      Pin 19     Data input to CC11xx
*  4 - SCK        14=D5    Pin 13      Pin 23     Clock pin
*  5 - MISO/GDO1  12=D6    Pin 12      Pin 21     Data output from CC11xx / serial clock from CC11xx
*  6 - GDO2       04=D2    Pin  2      Pin 22     Programmable output
*  7 - GDO0       ?        Pin  ?      NC         Programmable output
*  8 - CSN        15=D8    Pin 10      Pin 24     Chip select / (SPI_SS)
```
Note that CC11xx pin GDO0 is not used.
Also note that the GDO2 pin connected to pin 22 on an RPi. Change ```#define ITHO_IRQ_PIN``` in the example ino accordingly.

You should keep the wires to the CC11xx module as short as possible.

Beware that the CC11xx modules are 3.3V (3.6V max) on all pins!
This won't be a problem with a RPi as it has 3.3 V logic level, just make sure to connect Vdd top a 3.3 V pin and not 5 V!

Inspiration for rewriting the code to allow it to work on a RPi is mostly from the C1101 library made by SpaCeTeddy (https://github.com/SpaceTeddy/CC1101)

