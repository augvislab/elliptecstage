# elliptec_stage
A python class for Elliptec's linear stages from Thorlabs.

## Installation
``` pip install git+https://github.com/augvislab/elliptec_stage ```

## How to use
``` python
import elliptecstage

# Setup the motor
# Connect to the stage
stage = elliptecstage.ElloStage()

# One need to change motor parameters
stage.initialize_motor() # Default values are set

# initialize the position
stage.move_home()
command, data, address = stage.read_message_blocking_position_response()

# Move stage
pos = 2.0 # [mm]
stage.move_absolute(pos)
command, z, address = stage.read_message_blocking_position_response()

```
