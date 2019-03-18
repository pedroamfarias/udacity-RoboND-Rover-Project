## Project: Search and Sample Return
### Writeup - Pedro Augusto.

---


[//]: # (Image References)

[image1]: ./misc/threshed_img.PNG

[image3]: ./misc/autonomous_mode.PNG


### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
- Added the function `obj_thresh()` using OpenCV to get the gold color and "extract" the rock from the image
- Modified the function `color_thresh()` with np.diter to get a half bottom of the image. With that, we can short the navigable terrain to closer to the camera and prevents the Rover to mark a navigable terrain area outside of the map. 

![alt text][image1]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
- Modified `process_image()` adding points for perspective transform
- Modified `perspective_transform()` adding mask a output to help removing undesired ares (as showed in "Project Demo Video"
- Put on Red Channel the obstacles, on Green the Rock and on Blue the navigable terrain. Only add blue channel if have some data after `obj_thresh()`
 
    - [Result Video](./output/test_mapping.mp4)

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.
- Added 'self.rock_found', 'self.Rover_stuck_str_time' and 'self.rover_yaw_loop' in `drive_rover.py` to auxiliate in decision tree 
- Added Rover.mode stuck and get_rock.
    - stuck mode help Rover to get unstuck 
    - get_rock mode is activated when Rover sees a rock after `rock_map` in perception.py have some data. 
- Modified `decision_step()` to Rover explore the navigable terrain, get rock, unstuck if stuck and desviate of obstacles. 
- Modified perception.py similar to Notebook to define navigable terrain, find rocks and obstacles. 
    - If find rocks, update the Rover.nav_dists and Rover.nav_angles according to rock coordinates.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

- The Rover can get some rocks but in some cases cannot get it.
- Need to improve the algorithm to not remap areas already mapped: just avoid the areas that already mapped.
- After all, the Rover cannot back to home position: need to redesign the Decision Tree to Rover after mapped some % of the area back to initial position.
- Sometimes showed a error and the program stop to forward the data to python:
```
C:\Users\pedro\Anaconda3\envs\RoboND\lib\site-packages\numpy\core\fromnumeric.py:2920: RuntimeWarning: Mean of empty slice.
  out=out, **kwargs)
C:\Users\pedro\Anaconda3\envs\RoboND\lib\site-packages\numpy\core\_methods.py:85: RuntimeWarning: invalid value encountered in double_scalars
  ret = ret.dtype.type(ret / rcount)
```

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

- Screen resolution: 1024 x 768
- Windowed: Set
- Graphics quality: good
- Select monitor: Display 1(left)
- FPS: 
    - min: 14
    - max: 16 

![alt text][image3]


