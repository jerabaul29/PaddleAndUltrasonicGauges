# Automatic wave experiences

Some code used for performing automatic experiences about wave propagation in grease ice. If you use some of this content, please cite our article using this code:

*Experiments on wave propagation in grease ice: combined wave gauges and PIV measurements*, Jean Rabault, Graig Sutherland, Atle Jensen, Kai H Christensen, Aleksey Marchenko, ArXiv (2018).

## Content

- **ReadingGauges**: code (both Arduino Mega side and computer side) for reading the data from the Banner gauges.
- **PaddleActuator_versionDUE**: code (both Arduino Due side and computer side) for the command system of the paddle. Note that the Arduino Due core should be modified to use an extended RX buffer for receiving data from the computer, see for example https://github.com/jerabaul29/ArduinoDue .
- **Both_WavePaddle_Gauges**: Jupyter notebook for easily using both the paddle and the gauges. This will let you perform experiments in a fully automated way.
