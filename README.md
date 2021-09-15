A python class for Elliptec's linear stages from Thorlabs.

## Installation
``` pip install git+https://github.com/augvislab/elliptec_stage ```

## How to use
``` python
import elliptecstage

# Setup the motor
# Connect to the stage
with serial.Serial(port='COM3', baudrate=9600, timeout=0.2) as com:
    stage = ElloStage(com, 0)
    
    # One need to change motor parameters
    stage.initialize_motor()  # Default values are set
    
    # initialize the position
    stage.move_home()
    command, data, address = stage.read_message_blocking_position_response()
    print(f'{command}\t{data}\t{address}')
    
    # Move stage
    pos = 2.0  # [mm]
    stage.move_absolute(pos)
    command, z, address = stage.read_message_blocking_position_response()
    print('Measured position: ' + str(z))
```

## Resources
[**Communications Protocol**](https://www.thorlabs.de/Software/Elliptec/Communications_Protocol/ELLx%20modules%20protocol%20manual.pdf) from Thorlabs' website.
