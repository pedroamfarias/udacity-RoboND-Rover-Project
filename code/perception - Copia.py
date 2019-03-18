import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])

    # Reference: https://docs.scipy.org/doc/numpy/reference/arrays.nditer.html
    with np.nditer(above_thresh[:75], op_flags=['readwrite']) as it:
        for x in it:
            x[...] = False

    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def obj_tresh(img, low_thresh=(20, 98, 76), high_tresh=(30, 255, 255)):
    #hsv_thresh=(52, 98, 76)
    #hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    
    #gold_bgr = np.int8([[[6, 177, 151]]])
    #hsv_gold = cv2.cvtColor(gold_bgr)
    
    low_obj = np.array(low_thresh)
    #low_obj = np.array([hsv_thresh[0]-10, hsv_thresh[1]-50, hsv_thresh[2]-50 ])
    #upper_obj = np.array([hsv_thresh[0]+10, hsv_thresh[1]+50, hsv_thresh[2]+50 ])
    upper_obj = np.array(high_tresh)
    
    #print(low_obj)
    #print(upper_obj)
    
    mask = cv2.inRange(hsv, low_obj, upper_obj)

    
    #res = cv2.bitwise_and(img, img, mask=mask)
    
    return mask


# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
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

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
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

    #this mask will be used to remove black down part in future to identify obstacles. 
    # the focus is to map the navegable terrain and not unused black part at bottom.
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
    
    return warped, mask


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    
    dst_size = 5
    bottom_offset = 6
    image = Rover.img

       
    source = np.float32([[14, 140], [301,140], [200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset], # this "- bottom_offset" is due Y axes start at top left
                              [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                              [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                             ])
    
    
    
    # 2) Apply perspective transform
    
    warped, mask = perspect_transform(image, source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples

    threshed = color_thresh(warped, rgb_thresh=(160, 160, 160))
    obs_map = np.absolute(np.float32(threshed) - 1) * mask    
    
    rock_map = obj_tresh(warped)
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
        
    Rover.vision_image[:,:,0] = 255 * obs_map
    Rover.vision_image[:,:,1] = 255 * rock_map*-1
    Rover.vision_image[:,:,2] = 255 * threshed

    # 5) Convert map image pixel values to rover-centric coords
    
    xpix, ypix = rover_coords(threshed)
    xpix_obs, ypix_obs = rover_coords(obs_map)    
    
    # 6) Convert rover-centric pixel values to world coordinates

    #getting size of map
    world_size = Rover.worldmap.shape[0]
    
    # get map scale from pixel size (10 pixels in rover = 1 pixel in map)
    scale = 2 * dst_size
    
    #getting xpos, ypos, yaw from class Databucket()
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    
    #print(xpos)
    #print(ypos)
    
    yaw = Rover.yaw
    
    x_world, y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
    
    x_obs_world, y_obs_world = pix_to_world(xpix_obs, ypix_obs, xpos, ypos, yaw, world_size, scale)    
    
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    #Add red channel to each coordenate that have obstacle
    Rover.worldmap[y_obs_world, x_obs_world, 0] = 255
    
    # #Add blue channel to each coordenate that have navigable terrain 
    # Rover.worldmap[y_world, x_world, 2] = 255
    
    #keep blue the navigable terrain, overlapping the red channel
    nav_pix = Rover.worldmap[:,:,2] > 0
    Rover.worldmap[nav_pix, 0] = 0

    Rover.rock_found = rock_map.any()
    
    if  Rover.rock_found and not Rover.picking_up:
        
        rock_x, rock_y = rover_coords(rock_map)
        rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, xpos, ypos, yaw, world_size, scale)
            
        Rover.worldmap[rock_y_world, rock_x_world, : ] = 255          
        
        #get distance and angle of rock in rover_coords 
        dist_rock, angles_rock = to_polar_coords(rock_x, rock_y)

        Rover.nav_dists = dist_rock
        Rover.nav_angles = angles_rock
        

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    elif not Rover.rock_found:        
        dist, angles = to_polar_coords(xpix, ypix)
        #mean_dir = np.mean(angles)
    
        Rover.nav_dists = dist
        Rover.nav_angles = angles
        #print("Rover.nav_dists: {} \n Rover.nav_angles: {}".format(dist,angles))
        #print("Len Rover.nav_dists: {} \n Len Rover.nav_angles: {}".format(len(dist),len(angles)))
 
    #Add blue channel to each coordenate that have navigable terrain 
    Rover.worldmap[y_world, x_world, 2] = 255

    print("Rock found: {}   Rover Mode: {}  Samples Coll: {}   Samples Loc: {} rockmap.any(): {} Rover.nav_angles: {} Rover.vel: {}".format(Rover.rock_found, Rover.mode,
         Rover.samples_collected, Rover.samples_located, rock_map.any(), len(Rover.nav_angles), Rover.vel ))
    return Rover