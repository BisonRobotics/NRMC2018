#!/usr/bin/python
import cv2
import numpy as np
import sys
import random
import imutils

start_height = 1.5
obstacle_height = 2.94
obstacle_end_height = start_height + obstacle_height
mining_height = 2.94
arena_height_in_meters = start_height + obstacle_height + mining_height

start_width = 3.78
obstacle_width = 3.78
mining_width = 3.78
arena_width_in_meters = mining_width

average_rock_width_in_meters = .2
max_rock_width_in_meters = .3
min_rock_width_in_meters = .1

max_crater_width_in_meters = .3
min_crater_width_in_meters = .1

number_of_craters = 2
number_of_rocks = 3

max_cost = 255

crater_cost = max_cost
rock_cost = max_cost

def is_spot_filled (array, radius, x, y):
    x_entries = range (x -radius, x+radius)
    y_entries = range (y - radius, y+radius)
    is_filled = False
    for x in x_entries:
        for y in y_entries:
            if (array[x, y] != 0):
                is_filled = True
    return is_filled

def add_walls (array, wall_width, arena_width, arena_height):
    lowest_wall_y = range (0, arena_width-1)
    lowest_wall_x = range (0, wall_width)
    left_wall_y = range (0, wall_width)
    left_wall_x = range (0, arena_height-1)
    right_wall_y = range (arena_width-wall_width-1, arena_width-1)
    right_wall_x = range (0, arena_height-1)
    highest_wall_y = range (0, arena_width-1)
    highest_wall_x = range (arena_height - wall_width-1, arena_height-1)
    array[np.ix_(lowest_wall_x, lowest_wall_y)] = max_cost
    array[np.ix_(left_wall_x, left_wall_y)] = max_cost
    array[np.ix_(right_wall_x, right_wall_y)] = max_cost
    array[np.ix_(highest_wall_x, highest_wall_y)] = max_cost
    return array


def is_location_in_obstacle (radius, x, y, height_min, height_max, width):
    is_in_height = (x - radius) > height_min and (x+radius) < height_max
    is_in_width = (y - radius) > 0 and (y + radius) < width
    return (is_in_height and is_in_width)

def renormalize_list (array, array_min, array_max):
    array_min_1 = np.amin(array)
    array_max_1 = np.amax (array)
    for el in array:
      array[el] = array[el] - array_min_1
      array[el] = array[el]/array_max_1
      array[el] = array[el] * array_max
    return array


def fill_location (array, radius, val, x, y):
    for x_el in range (x-radius, x+radius):
      for y_el in range (y-radius, y+radius):
        if (((x_el-x)**2 + (y_el -y)**2) <  radius*radius):
          array[x_el][y_el] = val
    return array


if len (sys.argv) == 4:
    try:
        resolution = float(sys.argv[1])
        pixels_per_meter = 1.0/resolution
    except ValueError:
        print ("parameter one was not a float")
        quit ()

    try:
        file_name = sys.argv[2]
    except:
        print ("Error in argument 2: filename")
        quit ()
    try:
      seed = long (sys.argv[3])
      random.seed (seed)
    except:
      print ("error in argument 4: seed")
      quit ()
    height_in_pixels = int(pixels_per_meter * arena_height_in_meters)
    width_in_pixels = int(pixels_per_meter * arena_width_in_meters)
    obstacle_start_in_pixels = int(start_height * pixels_per_meter)
    obstacle_end_in_pixels = int(obstacle_end_height * pixels_per_meter)

    blank_costmap = np.zeros((height_in_pixels,width_in_pixels,1), np.uint8)

    average_rock_width_in_pixels = int(average_rock_width_in_meters * pixels_per_meter)
    max_rock_width_in_pixels = int(average_rock_width_in_meters * pixels_per_meter)
    min_rock_width_in_pixels = int(average_rock_width_in_meters * pixels_per_meter)

    max_crater_width_in_pixels = int(max_crater_width_in_meters * pixels_per_meter)
    min_crater_width_in_pixels =int( min_crater_width_in_meters * pixels_per_meter)

    while (True):
        x_location = random.randint (obstacle_start_in_pixels, obstacle_end_in_pixels)
        y_location = random.randint (0, width_in_pixels)
        radius = random.randint(min_crater_width_in_pixels,max_crater_width_in_pixels)
        if (is_location_in_obstacle(radius,x_location,y_location,obstacle_start_in_pixels,obstacle_end_in_pixels,width_in_pixels)):
            if (not is_spot_filled (blank_costmap,radius, x_location, y_location)):
                blank_costmap = fill_location(blank_costmap, radius, crater_cost, x_location, y_location)
                break
    while (True):
        x_location = random.randint (obstacle_start_in_pixels, obstacle_end_in_pixels)
        y_location = random.randint (0, width_in_pixels)
        radius = random.randint(min_crater_width_in_pixels,max_crater_width_in_pixels)
        if (is_location_in_obstacle(radius,x_location,y_location,obstacle_start_in_pixels,obstacle_end_in_pixels,width_in_pixels)):
            if (not is_spot_filled (blank_costmap,radius, x_location, y_location)):
                blank_costmap = fill_location(blank_costmap, radius, crater_cost, x_location, y_location)
                break

    while (True):
            x_location = random.randint (obstacle_start_in_pixels, obstacle_end_in_pixels)
            y_location = random.randint (0, width_in_pixels)
            radius = random.randint(min_crater_width_in_pixels,max_crater_width_in_pixels)
            if (is_location_in_obstacle(radius,x_location,y_location,obstacle_start_in_pixels,obstacle_end_in_pixels,width_in_pixels)):
                if (not is_spot_filled (blank_costmap,radius, x_location, y_location)):
                    blank_costmap = fill_location(blank_costmap, radius, rock_cost, x_location, y_location)
                    break

    while (True):
            x_location = random.randint (obstacle_start_in_pixels, obstacle_end_in_pixels)
            y_location = random.randint (0, width_in_pixels)
            radius = random.randint(min_crater_width_in_pixels,max_crater_width_in_pixels)
            if (is_location_in_obstacle(radius,x_location,y_location,obstacle_start_in_pixels,obstacle_end_in_pixels,width_in_pixels)):
                if (not is_spot_filled (blank_costmap,radius, x_location, y_location)):
                    blank_costmap = fill_location(blank_costmap, radius, rock_cost, x_location, y_location)
                    break

    while (True):
            x_location = random.randint (obstacle_start_in_pixels, obstacle_end_in_pixels)
            y_location = random.randint (0, width_in_pixels)
            radius = random.randint(min_crater_width_in_pixels,max_crater_width_in_pixels)
            if (is_location_in_obstacle(radius,x_location,y_location,obstacle_start_in_pixels,obstacle_end_in_pixels,width_in_pixels)):
                if (not is_spot_filled (blank_costmap,radius, x_location, y_location)):
                    blank_costmap = fill_location(blank_costmap, radius, rock_cost, x_location, y_location)
                    break
    add_walls(blank_costmap, 2,width_in_pixels,height_in_pixels)
#    rows, cols = blank_costmap.shape
    dst = imutils.rotate_bound (blank_costmap, 270)
    cv2.imwrite (file_name+'.bmp', dst)


else:
    print ("please enter 3 arguments after the call")
    print ("resolution filename seed")
