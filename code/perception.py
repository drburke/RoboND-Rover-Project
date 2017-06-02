import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def obj_thresh(img, rgb_thresh=(170, 140, 140)):
    
    
    #nav threshold
    nav_thresh = ((img[:,:,0] >= 160) \
                & (img[:,:,1] >= 160) \
                & (img[:,:,2] >= 160))
    
    
    #rock threshold
    rock_thresh = (img[:,:,0] > 110) \
                    & (img[:,:,1] > 110) \
                    & (img[:,:,2] < 50)
    
    # Create an array of zeros same xy size as img, but single channel
    navigable_select = np.zeros_like(img[:,:,0])
    obstacle_select = np.ones_like(img[:,:,0])
    rock_select = np.zeros_like(img[:,:,0])
    
    navigable_select[nav_thresh] = 1.
    rock_select[rock_thresh] = 1.
    obstacle_select = np.clip(obstacle_select - navigable_select - rock_select,0,1)
    
    obs_thresh = ((img[:,:,0] == 0) \
            & (img[:,:,1] == 0) \
            & (img[:,:,2] == 0))
    
    obstacle_select[obs_thresh] = 0.

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


# Get nearby pixels in field of view and 'visited'
def getNearishPixels(x,y,fov,visited):
    
    obs_dist = np.sqrt((x)**2 + (y)**2)
    indices_FOV = [obs_dist < fov]
    indices_visited = [obs_dist < visited]

    x_fov = x[indices_FOV]
    y_fov = y[indices_FOV]
    x_vis = x[indices_visited]
    y_vis = y[indices_visited]

    return x_fov, y_fov, x_vis, y_vis


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()

    # 1) Define source and destination points for perspective transform
    img = Rover.img

    dst_size = 5.
    scale = dst_size * 2.
    world_size = 200

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

    # Grab object thresholds from Rovers POV, can change what is displayed
    nav, obs, roc = obj_thresh(img)
    rover_POV = True

    # 4) Update Rover.vision
    if rover_POV:
        Rover.vision_image[:,:,0] = obs*255
        Rover.vision_image[:,:,1] = roc*255
        Rover.vision_image[:,:,2] = nav*255
    else:
        Rover.vision_image[:,:,0] = obstacle_select*255
        Rover.vision_image[:,:,1] = rock_select*255
        Rover.vision_image[:,:,2] = navigable_select*255
  



    # 5) Convert thresholded image pixel values to rover-centric coords
    xpix_nav, ypix_nav = rover_coords(navigable_select)
    xpix_obstacle, ypix_obstacle = rover_coords(obstacle_select)
    xpix_rock, ypix_rock = rover_coords(rock_select)
    

    fov_thresh = 50.
    visited_thresh = 30.
    xpix_obs_fov,ypix_obs_fov,xpix_obs_visited,ypix_obs_visited = getNearishPixels(xpix_obstacle,ypix_obstacle,fov_thresh-3.,visited_thresh)
    xpix_nav_fov,ypix_nav_fov,xpix_nav_visited,ypix_nav_visited = getNearishPixels(xpix_nav,ypix_nav,fov_thresh,visited_thresh)
    xpix_rock_fov,ypix_rock_fov,xpix_rock_visited,ypix_rock_visited = getNearishPixels(xpix_rock,ypix_rock,999,visited_thresh)


    # 6) Convert rover-centric pixel values to world coords
    rover_xpos = Rover.pos[0]
    rover_ypos = Rover.pos[1]
    rover_yaw = Rover.yaw

    if Rover.first_time == 1:
        Rover.first_time = 0
        Rover.initial_x = np.int_(rover_xpos)
        Rover.initial_y = np.int_(rover_ypos)
        Rover.rock_count = 0

    # keep track of where we've visited that is navigable
    worldmap = np.zeros((200, 200))
    visited_x_world, visited_y_world = pix_to_world(xpix_nav_visited, ypix_nav_visited, rover_xpos, 
                                    rover_ypos, rover_yaw, 
                                    world_size, scale)
    Rover.visitedmap[visited_y_world,visited_x_world] = 1

    

    # Get navigable pixel positions in world coords
    navigable_x_world, navigable_y_world = pix_to_world(xpix_nav_fov, ypix_nav_fov, rover_xpos, 
                                    rover_ypos, rover_yaw, 
                                    Rover.worldmap.shape[0], scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(xpix_obs_fov, ypix_obs_fov, rover_xpos, 
                                    rover_ypos, rover_yaw, 
                                    Rover.worldmap.shape[0], scale)
    rock_x_world, rock_y_world = pix_to_world(xpix_rock_fov, ypix_rock_fov, rover_xpos, 
                                    rover_ypos, rover_yaw, 
                                    Rover.worldmap.shape[0], scale)
    


    # 6) Update worldmap (to be displayed on right side of screen)
    # Only update if rover is appropriately untilted
    if (np.abs(Rover.pitch) < 0.6 or np.abs(Rover.pitch - 360) < 0.6) and (np.abs(Rover.roll)  < 1.0 or np.abs(Rover.roll - 360)  < 1.0):
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

        
    
    Rover.worldmap[:,:,0] -= Rover.visitedmap
    Rover.worldmap = np.clip(Rover.worldmap,0,255)

    obstacles = Rover.worldmap[:,:,0]
    navigable = Rover.worldmap[:,:,2]
    obs_nav = np.clip(obstacles-navigable - Rover.visitedmap,0.,1.)
    new_territory = np.clip(np.ones((200,200),dtype=np.float) - obs_nav,0.,1.) * 1. - Rover.visitedmap * 0.9
    Rover.heatmap = new_territory

    # if we've found all of the rocks set home base as the highest point
    if Rover.rock_count >= Rover.total_rocks:
        Rover.heatmap[Rover.initial_y,Rover.initial_x] = 100    

    Rover.heatmap = cv2.blur(Rover.heatmap,(14,14)) + cv2.blur(Rover.heatmap,(25,25)) + cv2.blur(Rover.heatmap,(6,6))
    tmp = Rover.heatmap - obs_nav * Rover.heatmap 
    cv2.normalize(tmp, tmp, 0, 255, cv2.NORM_MINMAX)
    Rover.heatmap = tmp*tmp


    Rover.heat = [Rover.heatmap[np.int_(xpix_nav_fov[i]),np.int_(ypix_nav_fov[i])] for i in range(len(ypix_nav_fov))]
    Rover.heat_dists, Rover.heat_angles = to_polar_coords(xpix_nav_fov,ypix_nav_fov)

    Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix_nav_fov,ypix_nav_fov)
    Rover.rock_dists, Rover.rock_angles = to_polar_coords(xpix_rock_fov,ypix_rock_fov)
    Rover.obs_dists, Rover.obs_angles = to_polar_coords(xpix_obs_fov,ypix_obs_fov)

    close_indices = [Rover.obs_dists < 7.4]
    Rover.close_obs = Rover.obs_dists[close_indices]
    return Rover