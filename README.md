# CNC-Pendant-Firmware

This is firmware to run on an STM32F411CEU6 (Black Pill) to interface a popular style of wired CNC pendant to the PanelDue port of Duet electronics. Build it using Arduino. 

Pendant to Arduino Pro Micro wiring:

```
ProMicro Pendant   Wire colours
VCC      +5V       red
GND      0V,       black
         COM,      orange/black
         CN,       blue/black
         LED-      white/black

PB12       A         green
PB13       B         white
PB14       X         yellow
PB15       Y         yellow/black
PA8        Z         brown
PA11       4         brown/black
PA12       LED+      green/black
PA15       STOP      blue
PB3        X1        grey
PB4        X10       grey/black
PB5        X100      orange

NC       /A,       violet
         /B        violet/black
```

Arduino to Duet PanelDue connector wiring (3- or 4-core cable):
```
ProMicro Duet
VCC      +5V (red wire)
GND      GND (yellow wire
TX1/D0   Through 6K8 resistor to URXD, also connect 10K resistor between URXD and GND (blue wire from resistor junction to Duet URXD0)
```

To connect a PanelDue as well (the pendant passes the PanelDue commands through):
```
PanelDue +5V to +5V/VCC (red wire)
PanelDue GND to GND (yellow wire)
PanelDue DIN to Duet UTXD or IO_0_OUT (green wire)
PanelDue DOUT to Arduino RX0 (blue wire of PanelDuet cable to green wire of pendant cable)
```

For wiring differences and hardware changes needed if using an Arduino Nano, see the comments at the start of the CNC-pendant.ino file.

