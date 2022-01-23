# stm_uv12

This repository cotains the kicad files and code for a UVx irradiator designed to fit over a standard 12 well plate.
The controller is powered by a single protected 18650 Li-ion cell, each row of LEDs (3 rows of 4 LEDs) has a R1218x DC/DC converter for control of LED brightness.
Approximate doses (J/m^2) have been calculated using the expected LED outputs and are delivered at a specified rate using the controller.

## PCB Files

UVA_12W_LED_CONTROL: stm32 control board with 16x2 OLED screen (16pin connector).

UVA_12W_LED: board for use with 4x VLMU1610 UVA LEDs.

UVC_12W_LED: board for use with 4x SML-LXF3535UVC UVC LEDs.
