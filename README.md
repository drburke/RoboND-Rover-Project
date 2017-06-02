## Project: Search and Sample Return
[//]: # (Image References)

[image1]: ./threshed.jpg
[image2]: ./heatmap.jpg

### Writeup / README

#### Rover Simulation Settings :
- Screen Resolution : 1024 x 768
- Graphics Quality : Good
- FPS : ~20

** on updating all files runs up to 50fps.  Seems (initially) more stable, I've increased the max speed a bit.

### Notebook Analysis

The main goal is to identify navigable terrain, obstacles and rocks.  Navigable terrain I found was doing a pretty good job with RGB values above 160.  Obstacles are

For the rocks I found that RG values above 110 and G values above 50 worked very well.  

Rocks in this simulation are mostly yellow and not terribly dark.  A simple RGB limit is R+G > 110, B < 50, the key trade being lots of reds and greens, not much blue.

In the transformed images there's a lot of black that can be recognized as obstacles despite being outside of the field of view.  I've removed black from the obstacles.  Also for nice imaging removed rocks from obstacles and navigable terrain.

Here is a nice image of this detection.

![threshold rock image][image1]

#### Mapping and Perception

First we want to warp the image from the rovers point of view to be top view / cartesian x-y space.  For this we must map matching squares to define an image transform.  Set 1 meter to be 10x10 pixels at the worldmap.  Define points of a warped square in robo image to square in world map, use cv2 to transform perspective.

Next, find the navigable, obstacle and rocks using the warped / top down images.  This uses the threshold function described above.  I also threshold the POV images as they are sometimes interesting to look at.

Convert warped images into x and y pixels with respect to rover location and direction.  'Rover-centric' coordinates.  I do this for all three sets of coordinates : navigable, obstacle, rock.

From these I split each group into pixels close enough to be considered 'visited' and those that are close enough to be in my field of view and are most likely accurate.

Transform these rover centric pixels to world map coordinates using the known global rover location, and yaw/angle.

I have created a map of pixels that are simply 'visited' in that the rover has seen them up close in its FOV.  Set the visited map to 1's for the navigable terrain.

I could do something fancy here too with rocks that have been seen but not yet picked up, would help with missing rocks when they are only briefly seen.

Update world map by '1' for each of the three types.  Store world map in a 3 channel array, could be later drawn using three RGB channels.  Only record these values if the rover pitch and yaw are close to zero.

Points in FOV are also converted to distances and angles, this is including the heat map values that would be in the FOV.

Last I've tried to do something a bit more fancy but also simple.  It seems to work decently.  Key is to create a full map where the pixel values are how desirable it would be to go visit that pixel.  All pixels start off as valuable. Visited pixels become less valuable, but still > 0.  The heat map is then blurred a few different blur levels, ideally to encourage avoiding close obstacles and moving to new areas that are close and far away.

This world map and heat map, partially filled, can be seen in the video and image below.  Heat map being greyscale and world map RGB.


![partially filled world map][image2]

### Autonomous Navigation

This was an interesting function to write.  I can imagine a few different ways to tackle this problem.  Here is one of them :

I'll try to explain the approach instead of simply walking through the if statements.  

#### Stop
If we're 'stopped' but still rolling, kill throttle and set the breaks.  Stay stopped.

If stopped and there isn't much navigable terrain in front of us, or there are too many close objects: rotate.  Otherwise we're done stopping, continue on.

#### Forward
If there are too many close objects in front of us, stop and rotate.  If throttle is up but we're not moving then increment a counter.  If this gets too high we're probably 'stuck'.  Also stop if there isn't any navigable terrain ahead.  This is similar to the obstacles above.

Moving forward, find a new angle based on the navigable terrain that is in the FOV and close enough.  Weight this angle by the heat map calculated previously.  I also steer just to the right of this ideal angle to keep the rover near walls and run a specific direction around the terrain.

Damp the steering angle by alpha, experimentally derived.  This adds a 'P' term.  Could later add an 'ID'.  This is to prevent the rover from oscillating.  When the framerate started skipping or updating less frequently the rover returns to oscillating.  

If angle is high and we're moving too fast, slow down.

#### Stuck
If we're stuck, first rotate left (for consistency in mapping), then back up in the direction that maximizes viewable navigable terrain.

#### Rocks
If at any point the rover sees a rock, immediately slow down if moving too fast and drive towards the rock slowly.  Once there pick it up and increment a rock counter.  I found the rocks weren't always counted by the other rock counter.  Sometimes the rover attempts to drive over rough or obstacle terrain to get to a rock, then gets stuck, then runs away.  This could be improved with better or actual path mapping and planning.  Or creating a rock + object based heat map.

Once done picking up the rock, back up- this should put us back in the same direction that we were going.  Ideally the heat map would do the same.

Once all rocks have been picked up, to return, set the starting points as having high value on the heat map, once there stop.

### Thoughts

All in all the rover does pretty well.  With a bit of time it can map the entire world and find all of the rocks- with few exceptions.  I seem to have stability problems with framerate and such when the rover gets too fast.  This prevented me from speeding it up too much.  

I have mentioned above a few improvements that could be made.  The best would probably be to add full path mapping and planning.  
