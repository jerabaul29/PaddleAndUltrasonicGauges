# Automatic wave experiences

Some code used for performing automatic experiences about wave propagation in grease ice.

## Content

- **ReadingGauges**: code (both Arduino Mega side and computer side) for reading the data from the Banner gauges.
- **PaddleActuator_versionDUE**: code (both Arduino Due side and computer side) for the command system of the paddle. Note that the Arduino Due core should be modified to use an extended RX buffer for receiving data from the computer, see for example https://github.com/jerabaul29/ArduinoDue .
- **Both_WavePaddle_Gauges**: Jupyter notebook for easily using both the paddle and the gauges. This will let you perform experiments in a fully automated way.
