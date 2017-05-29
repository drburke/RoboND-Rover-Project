import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def obj_thresh(img, rgb_thresh=(170, 140, 140)):
    
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Create an array of zeros same xy size as img, but single channel
    navigable_select = np.zeros_like(img[:,:,0])
    obstacle_select = np.zeros_like(img[:,:,0])
    
    lower_nav = np.array([10,10,190])
    upper_nav = np.array([200,220,255])

    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    # above_thresh = (img[:,:,0] > rgb_thresh[0]) \
    #             & (img[:,:,1] > rgb_thresh[1]) \
    #             & (img[:,:,2] > rgb_thresh[2])

    nav_thresh = ((img[:,:,0] >= 160) \
                    & (img[:,:,1] >= 160) \
                    & (img[:,:,2] >= 160))

    ones = np.ones_like(img[:,:,0]) * 1
    mask = cv2.inRange(img_hsv, lower_nav, upper_nav)
    navigable_select = cv2.bitwise_and(ones,ones, mask= mask)
    navigable_select[nav_thresh] = 1

    obstacle_select = 1 - navigable_select
    
    rock_thresh = (img[:,:,0] > 110) \
                    & (img[:,:,1] > 110) \
                    & (img[:,:,2] < 50)


    rock_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met

    # Index the array of zeros with the boolean array and set to 1
    rock_select[rock_thresh] = 1.

    # ones = np.ones_like(img[:,:,0]) * 1
    # lower_rock = np.array([10,70,120])
    # upper_rock = np.array([35,255,240])
    # # Threshold the HSV image to get only blue colors
    # mask = cv2.inRange(img_hsv, lower_rock, upper_rock)
    #cv2.bitwise_and(ones,ones, mask= rock_thresh)
    # navigable
    all_black = np.zeros_like(img[:,:,0])
    black_indices = ((img[:,:,0] == 0) \
                    & (img[:,:,1] == 0) \
                    & (img[:,:,2] == 0))

    rock_select = np.clip(rock_select,0,1)
    navigable_select = np.clip(navigable_select,0,1)
    navigable_select = np.clip(navigable_select-rock_select,0,1)#cv2.bitwise_or(rock_select,navigable_select)
    obstacle_select = np.clip(obstacle_select,0,1)
    obstacle_select = np.clip(obstacle_select-rock_select,0,1)
    

    navigable_select[black_indices] = 0
    obstacle_select[black_indices] = 0

    # (h, w) = obstacle_select.shape[:2]
    # obstacle_select[:int(h/2),:] = 0
    # Return the binary image
    return navigable_select, obstacle_select, rock_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):

    
    # yaw angle is recorded in degrees so first convert to radians
#     print(yaw)
#     print(np.pi)
#     print(type(yaw))
#     print(type(np.pi))
    yaw_rad = float(yaw) * np.pi / 180
    x_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    y_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)

    # Return the result  
    return x_rotated, y_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    

    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + (xpix_rot / scale))
    ypix_translated = np.int_(ypos + (ypix_rot / scale))
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()

    # 1) Define source and destination points for perspective transform
    
    img = cv2.blur(Rover.img,(5,5))
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      ])

    # 2) Apply perspective transform
    warped = perspect_transform(img, source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable_select, obstacle_select, rock_select = obj_thresh(warped)

    nav, obs, roc = obj_thresh(img)
    # 4) Update Rover.vision
    Rover.vision_image[:,:,0] = obs*255
    Rover.vision_image[:,:,1] = roc*255
    Rover.vision_image[:,:,2] = nav*255
    
  
    # 5) Convert thresholded image pixel values to rover-centric coords
    xpix, ypix = rover_coords(navigable_select)
    xpix_obstacle, ypix_obstacle = rover_coords(obstacle_select)
    xpix_rock, ypix_rock = rover_coords(rock_select)
    
    # 6) Convert rover-centric pixel values to world coords
    rover_xpos = Rover.pos[0]
    rover_ypos = Rover.pos[1]
    rover_yaw = Rover.yaw


    obs_dist = np.sqrt((xpix_obstacle)**2 + (ypix_obstacle)**2)
    indices_close = obs_dist < 65.
    xpix_obstacle = xpix_obstacle[indices_close]
    ypix_obstacle = ypix_obstacle[indices_close]
    # obs_dist = np.sqrt((rover_xpos-xpix_obstacle)**2 + (rover_ypos-ypix_obstacle)**2)
    # indices_close = obs_dist < 30
    # xpix_obstacle = xpix_obstacle[indices_close]
    # ypix_obstacle = ypix_obstacle[indices_close]

    # Generate 200 x 200 pixel worldmap
    worldmap = np.zeros((200, 200))
    scale = 10
    # Get navigable pixel positions in world coords
    navigable_x_world, navigable_y_world = pix_to_world(xpix, ypix, rover_xpos, 
                                    rover_ypos, rover_yaw, 
                                    worldmap.shape[0], scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(xpix_obstacle, ypix_obstacle, rover_xpos, 
                                    rover_ypos, rover_yaw, 
                                    worldmap.shape[0], scale)
    rock_x_world, rock_y_world = pix_to_world(xpix_rock, ypix_rock, rover_xpos, 
                                    rover_ypos, rover_yaw, 
                                    worldmap.shape[0], scale)
    
    
    # 6) Update worldmap (to be displayed on right side of screen)
    if (np.abs(Rover.pitch) < 0.6 or np.abs(Rover.pitch - 360) < 0.6) and (np.abs(Rover.roll)  < 1.0 or np.abs(Rover.roll - 360)  < 1.0):
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 0] = 0
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

        Rover.worldmap = np.clip(Rover.worldmap,0,255)
    
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix,ypix)
    Rover.rock_dists, Rover.rock_angles = to_polar_coords(xpix_rock,ypix_rock)
    Rover.obs_dists, Rover.obs_angles = to_polar_coords(xpix_obstacle,ypix_obstacle)

    close_indices = Rover.obs_dists < 7.4
    Rover.close_obs = Rover.obs_dists[close_indices]
    return Rover