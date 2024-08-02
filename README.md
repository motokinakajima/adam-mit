# adam-mit
MITで走るプログラム！！

# elevator_controller

elevator controller is a class that is created just for the racecar2024 grandprix. gets the image and scan as an input, outputs speed and angle
## example

```python
import sys
import numpy as np
from nptyping import NDArray
from enum import Enum
import matplotlib.pyplot as plt

sys.path.insert(1, 'your/path/to/resources')

from mode_manage import *
from elevator_controller import *


sys.path.insert(1, 'your/path/to/library')

import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()

RED1 = ((0, 100, 100), (18, 255, 255))  # Lower red range
RED2 = ((160, 100, 100), (180, 255, 255))
BLUE = ((68, 101, 124), (129, 229, 206))

UPPER_CROP = ((360,0),(420, 640))
LOWER_CROP = ((420, 0), (480, 640))

#inisializing elevator_controller
#elevatorController(kp, ki, kd)
elevator_controller = elevatorController(0.01, 0.0, 0.005)

mode_manager = ModeManager()

def start():
    #resets the queue from start
    elevator_controller.reset()

def update():
    image = rc.camera.get_color_image()
    scan = rc.lidar.get_samples()
    
    #getting speed and angle based on elevator movements
    speed, angle = elevator_controller.update(image, scan, [BLUE], [RED1, RED2])
    
    #using this because I have not adjusted my PID yet
    angle = np.clip(angle, -1, 1)
    
    rc.drive.set_speed_angle(speed, angle)

def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
```

## variables

I have set some variables that are changable from main.py

### wait_distance
this indicates how far the car stops from the elevator.

default:
```150```

how to modify:
```python
elevatorController(kp, ki, kd, wait_distance=200)
```
### elevator_distance
this indicates how close to the wall the car gets when getting on the elevator.

default:
```80```

how to modify:
```python
elevatorController(kp, ki, kd, elevator_distance=100)
```
### bypass_speed
this indicates how fast the car goes in the rainbow road before entering the division.

default:
```1.0```

how to modify:
```python
elevatorController(kp, ki, kd, bypass_speed=0.8)
```
### ar_follow_speed
this indicates how fast the car goes after passing the division, towards the armarker on the wall.

default:
```0.5```

how to modify:
```python
elevatorController(kp, ki, kd, ar_follow_speed=0.8)
```
### get_onto_speed
this indicates how fast the car goes when getting onto the elevator.

default:
```1.0```

how to modify:
```python
elevatorController(kp, ki, kd, get_onto_speed=0.8)
```
### divider_target_x
this indicates the goal x coordinate of the armarker on the devision. this shoule be bigger than the width of the image to go to the left side of the division.

default:
```400```

how to modify:
```python
elevatorController(kp, ki, kd, divider_targetx=380)
```