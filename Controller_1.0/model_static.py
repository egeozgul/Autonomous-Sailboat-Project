# -*- coding: utf-8 -*-
"""
Created on Sat Nov 14 20:35:19 2020

@author: kamil
"""
from math import cos, pi, sqrt, atan


""" Returns distance in meters, [x, y, direct]"""
def distance(lat, lon, target_lat, target_lon):
    y = target_lon - lon
    y *= 111.32 * pow(10,4)
    x = target_lat * (40075 * pow(10,4)) * cos(target_lat) / 360
    x -= lat * (40075 * pow(10,4)) * cos(target_lat) / 360
    return [x , y, sqrt(x * x + y * y)]

""" 
Transforms cartesian coordinates to polar theta.
"""
def get_theta(x, y):
    angle = 0;
    if x == 0:
        if y < 0:
            angle = 180;
    else:
        angle = atan(y/x) * 180 / pi
    if (x < 0):
        angle += 180
    if (angle < 0):
        angle += 360
    return angle;

"""
Hard-coded rule-based sailing model
Input:
    starting coords, target coords, 
    wind_dir (degrees), boat angle (degrees).
    0 degrees represents east.

output:
    [rudder, sail]
    rudder is 0 - 180 degrees. 90 is neutral, 0 is left, 180 is right.
    sail is 0-100. It represents freedom of main sheet.
    0 is fully tightened, 100 is fully released.


"""
def sail(lat, lon, target_lat, target_lon, wind_dir, boat_angle):

    """ Heading is desired sailboat travel angle """
    [x,y,dist] = distance(lat,lon,target_lat,target_lon);
    heading = get_theta(x,y);
    
    wind_source = wind_dir + 180
    if (wind_source > 360):
        wind_source -= 360;
    
    # Tack if sailing directly upwind
    # 45 degree cone into wind
    if (heading + 360 > wind_source + 337
        and heading + 360 < wind_source + 383):
        if (heading - wind_source + 22.5 > 22.5):
            heading = wind_source - 22.5;
        else:
            heading = wind_source + 22.5;
    
    # Do not sail directly downwind
    # 30 degree cone downwind
    if (heading + 360 > wind_dir + 345 
        and heading + 360 < wind_dir + 375):
        if (heading - wind_dir + 15 > 15):
            heading = wind_dir - 15;
        else:
            heading = wind_dir + 15;
    
    rudder = 90;
    #boat angle should equal heading.
    if (boat_angle > heading):
        rudder -= (heading - boat_angle)
        rudder = max(0,rudder);
    else:
        rudder += (heading - boat_angle)
        rudder = min(180,rudder);
    
    dot = cos(abs(wind_dir - boat_angle))
    norm_dot = (dot + 1)/2
    sail = 100 * norm_dot;
    
    return [rudder, sail];
    

if __name__ == "__main__":
    """Medford lat,lon"""
    lat = 42.326542
    lon = -122.872192
    target_lat = lat + 0.000001
    target_lon = lon + 0.000001
    boat_angle = 90
    """ Where wind comes from"""
    wind_angle = 95
    
    rudder, sail = sail(lat,lon,target_lat,target_lon,wind_angle, boat_angle)
    [deltax,deltay,dist] = distance(lat, lon, target_lat, target_lon)
    
    print("Distance: {:.2f} m".format(dist))
    print("Destination Angle: {:.2f} deg".format(get_theta(deltax,deltay)))
    print("Boat angle: {}".format(boat_angle))
    print("Wind angle:  {}".format(wind_angle))
    print("\nOptimal Prediction")
    print("Rudder: {:.2f} deg , Sail: {:.2f} % free".format(rudder,sail))
