
RoboND Write-up


-- Color Thresholding --

Notebook section : Color Thresholding

Main goal is to identify navigable terrain, obstacles and rocks.  Navigable terrain I found was initially labelling some of the sky as terrain.  To improve this I have added upper and lower bounds in HSV space.  HSV allows to specify brightness and specific colours which can be challenging with RGB.  Two limits were found which seemed to decently well split sky from navigable terrain.  This still left some 'whites' out so a second threshold of RGB > 230 was added.

Obstacle terrain is simply 1-navigable terrain.  

Rocks in this simulation are mostly yellow and not terribly dark.  A simple RGB limit is R+G > 110, B < 50, the key trade being lots of reds and greens, not much blue.


-- Process Image --

Notebook section - Process Image

First must map an image transform to warp perspective of robot to x-y cartesian.  Set 1 meter to be 10x10 pixels at the worldmap.  Define points of a warped square in robo image to square in world map, use cv2 to transform perspective.

Next, find the navigable, obstacle and rocks using the warped / top down images.  Convert these into x and y pixels with respect to rover location and direction.  'Rover-centric' coordinates.

Transform these rover centric pixels to world map coordinates using the known global rover location, and yaw/angle.

Update world map by '1' for each of the three types.  Store world map in a 3 channel array, could be later drawn using three RGB channels.  I found that navigable is a bit more trustworthy metric and obstacle, so if a pixel is navigable then also decrement obstacle in that same location.  Also clip the worldmap to be safe.



-- Autonomous Navigation and Mapping - 

perception.py  -  perception_step()

This is mostly a repeat of the process image step.  img is copied from Rover, warped, thresholded, then updated into the rover vision_image.  Images are mapped to rover centric coords, then mapped using rover current location to world coordinates.

The perspective warping assumes that the relationship between rover view and global map stays constant.  If the rover is tilted too much in any direction this mapping is no longer accurate.  With some effort could map tilts/rolls to perspective warp, but for here the images are simply not mapped if there is too much tilting or rolling.

Lastly I copy the navigable dists and angles to the Rover data, and the Rock dists angles to the rover polar coordinates.


decision.py - decision_step()

Here the decision tree has grown a bit to include slightly better navigation and rock picking up.  

Stuck : 
First - stuck mechanism.  If the rover has throttle forward but no velocity a counter is increased.  If the counter gets too high we assume the rover is stuck.  This sets the steering opposite its current direction, and reverses throttle for a set duration.  After this the mode is set from 'stuck' to 'stop' and the counter is reset.

Rock :
Next, if the rover is not stuck (sometimes it gets stuck while being able to see a rock, this isn't helpful) and it sees a rock (len(rock_angles) > 5) it immediately drops speed and steers towards the rock.  Once close enough to the rock it requests a rock pickup.  Since rocks are normally at the edge and can interfere with previous navigation/direction, once a rock is picked up the reversal action from 'stuck' is implemented to back us up a bit.

Forward :
If rover is moving forward, use the mean rover navigable angles for its direction.  Slight change here -  I want the rover to hug the walls.  To do this the steering angle set is slightly off the mean.  Using steering as the mean of navigable angles also introduces some oscillation.  A simple fix is to add in a proportional term (P in PID), thus this angle is reduced by 'alpha' which I set to 0.5.  This adds an amount of smoothing and removes the oscillations.

Stop :
This section has not been edited.