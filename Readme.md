# Automatic wave experiences

Some code used for performing automatic experiences about wave propagation in grease ice. If you use some of this content, please cite our article using this code:

Rabault, Jean, et al. "Experiments on wave propagation in grease ice: combined wave gauges and particle image velocimetry measurements." Journal of Fluid Mechanics 864 (2019): 876-898.

Note that it is available as a preprint on ArXiv an ResearchGate: https://www.researchgate.net/publication/331083269_Experiments_on_wave_propagation_in_grease_ice_Combined_wave_gauges_and_particle_image_velocimetry_measurements .

## Content

- **ReadingGauges**: code (both Arduino Mega side and computer side) for reading the data from the Banner gauges.
- **PaddleActuator_versionDUE**: code (both Arduino Due side and computer side) for the command system of the paddle. Note that the Arduino Due core should be modified to use an extended RX buffer for receiving data from the computer, see for example https://github.com/jerabaul29/ArduinoDue .
- **Both_WavePaddle_Gauges**: Jupyter notebook for easily using both the paddle and the gauges. This will let you perform experiments in a fully automated way.
