# 3D-Printer

## Overview

An Anet A8 3D printer was stripped of its controller board and its components connected to an Arduino Uno as shown in the following schematic.
![Schematic](https://user-images.githubusercontent.com/109980480/200149283-442b8658-7d8f-4dd2-8172-1a15c7749959.jpg)

|Part ID|Part Name|Part Number||Part ID|Part Name|Part Number|
|---|---|---|---|---|---|---|
|C1|0.1uF|||U4|Stepper Driver|A4988|
|C2|0.1uF|||U5|Stepper Driver|A4988|
|C3|0.1uF|||U6|X-Axis Stepper|42SHDC3025-24B|
|C4|0.1uF|||U7|Y-Axis Stepper|42SHDC3025-24B|
|C5|0.1uF|||U8|Z-Axis Stepper|42SHDC3025-24B|
|C6|100uF|||U9|Z-Axis Stepper|42SHDC3025-24B|
|C7|100uF|||U10|Extruder Stepper|42SHDC3025-24B|
|C8|100uF|||U11|X-Min Stop|10T85|
|C9|100uF|||U12|Y-Min Stop|10T85|
|C10|10uF|||U13|Z-Min Stop|10T85|
|M1|IRF520|||U14|Power Module|BIQU-3D0109|
|R1|100k|||U15|Hot End|ANET-0029|
|U1|Arduino Uno|Arduino-A000066||U16|Thermistor|NTC 3960 100k|
|U2|Stepper Driver|A4988||U17|USB to Serial|OSEPP FTD-01|
|U3|Stepper Driver|A4988|

A STL file (cube.stl) was created of a 1.25-inch cube with 0.125-inch walls and a 1-inch by 0.125-inch opening on top. The STL was converted into g-code (cube.gcode) using Cura.

Knowing Repetier Host would be used to stream the g-code from a PC to the Arduino Uno, code was created for the Arduino Uno to receive g-code through its USART and parse the subset of g-code commands found in the g-code file.

The following g-codes from the file were implemented: G0 (rapid linear move), G1 (linear move), G28 (move to origin), and M109 (set extruder temperature and wait). In addition, M105 (get extruder temperature) was implemented to display the extruder’s temperature in Repetier Host. 

The following settings are hard-coded: units are in millimeters, all positioning is absolute, fans are always on and feed rate is constant. Since feed rate is constant, parameter ‘F’ is not used and, G0 and G1 are executed in the same manner. Additional restrictions to G0 and G1 functionalities are as follows: the length of filament to extrude is calculated (one pulse to the extruder stepper motor for every thirty pulses to the movement stepper motors), filament is only extruded if there is movement and the extruder direction is never reversed. Since the extruder direction is never reversed, the last two G1 lines in cube.gcode were commented out.

For M105 and M109, a one second timer triggers the ADC conversion of the thermistor voltage and the temperature is determined using the calculations found at https://www.allaboutcircuits.com/projects/measuring-temperature-with-an-ntc-thermistor/.

![Controller_Hardware](https://user-images.githubusercontent.com/109980480/200149310-b84655fb-53e6-48f4-8c5f-df0707306f9e.jpg)
![Anet_A8](https://user-images.githubusercontent.com/109980480/200149320-d698ff78-b1f5-4c41-bec4-8fede70c0f2f.jpg)
![3D_Object](https://user-images.githubusercontent.com/109980480/200149321-3baf5a7f-bafa-422b-a1e4-3604396c0f67.jpg)
