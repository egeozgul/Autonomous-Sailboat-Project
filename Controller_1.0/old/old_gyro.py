import smbus
import math
import time
 
# Register
power_mgmt_1 = 0x6b;
power_mgmt_2 = 0x6c;

global gyroscope_xout, gyroscope_yout, gyroscope_zout
global acceleration_xout, acceleration_yout, acceleration_zout
global acceleration_xout_norm, acceleration_yout_norm, acceleration_zout_norm
 
def read_byte(reg):
    return bus.read_byte_data(address, reg)
 
def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value
 
def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
 
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
 
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

def get_z_rotation(x,y,z):
    radians = math.atan2(z, dist(x,y))
    return math.degrees(radians)

def __update__():
    global gyroscope_xout, gyroscope_yout, gyroscope_zout
    global acceleration_xout, acceleration_yout, acceleration_zout
    global acceleration_xout_norm, acceleration_yout_norm, acceleration_zout_norm
    global mag;

    mag = read_word_2c(0x09);

    gyroscope_xout = read_word_2c(0x43)
    gyroscope_yout = read_word_2c(0x45)
    gyroscope_zout = read_word_2c(0x47)

    acceleration_xout = read_word_2c(0x3b)
    acceleration_yout = read_word_2c(0x3d)
    acceleration_zout = read_word_2c(0x3f)
    
    acceleration_xout_norm = acceleration_xout / 16384.0
    acceleration_yout_norm = acceleration_yout / 16384.0
    acceleration_zout_norm = acceleration_zout / 16384.0

def get_angle():
    return 0

def get_values():
    global gyroscope_xout, gyroscope_yout, gyroscope_zout
    global acceleration_xout, acceleration_yout, acceleration_zout
    global acceleration_xout_norm, acceleration_yout_norm, acceleration_zout_norm

    """
    return [gyroscope_xout, gyroscope_yout, gyroscope_zout,
            acceleration_xout, acceleration_yout, acceleration_zout,
        acceleration_xout_norm, acceleration_yout_norm, acceleration_zout_norm]
    """

    line = "{%5d},{%5d},{%5d},".format(gyroscope_xout, 
                                    gyroscope_yout, gyroscope_zout);
    line += "{%5d},{%5d},{%5d},".format(acceleration_xout, 
                                    acceleration_yout, acceleration_zout); 
    line += "{%5d},{%5d},{%5d},".format(acceleration_xout_norm, 
                            acceleration_yout_norm, acceleration_zout_norm); 
    
    line += "{},{}".format(get_x_rotation(acceleration_xout_norm, 
                    acceleration_yout_norm, acceleration_zout_norm),
                    get_y_rotation(acceleration_xout_norm, 
                    acceleration_yout_norm, acceleration_zout_norm))
    
    return line;

def printvalues():
    global gyroscope_xout, gyroscope_yout, gyroscope_zout
    global acceleration_xout, acceleration_yout, acceleration_zout
    global acceleration_xout_norm, acceleration_yout_norm, acceleration_zout_norm
    global mag;

    print("Gyroscope")
    print("--------")

    print("gyroscope_xout: ", ("%5d" % gyroscope_xout), " norm: ", (gyroscope_xout / 131))
    print("gyroscope_yout: ", ("%5d" % gyroscope_yout), " norm: ", (gyroscope_yout / 131))
    print("gyroscope_zout: ", ("%5d" % gyroscope_zout), " norm: ", (gyroscope_zout / 131))

    print("Accelerameter")
    print("---------------------")
    
    print("acceleration_xout: ", ("%6d" % acceleration_xout), " norm: ", acceleration_xout_norm)
    print("acceleration_yout: ", ("%6d" % acceleration_yout), " norm: ", acceleration_yout_norm)
    print("acceleration_zout: ", ("%6d" % acceleration_zout), " norm: ", acceleration_zout_norm)
    
    print("X Rotation: " , get_x_rotation(acceleration_xout_norm, acceleration_yout_norm, acceleration_zout_norm))
    print("Y Rotation: " , get_y_rotation(acceleration_xout_norm, acceleration_yout_norm, acceleration_zout_norm))
    print('MAG {}'.format(mag))
def __sense__():
    while(1):
        __update__();

def get_xyz():
    global gyroscope_xout, gyroscope_yout, gyroscope_zout
    return [gyroscope_xout, gyroscope_yout, gyroscope_zout]

def get_accel():
    global acceleration_xout, acceleration_yout, acceleration_zout
    return [acceleration_xout, acceleration_yout, acceleration_zout]

def get_norm_accel():
    global acceleration_xout_norm, acceleration_yout_norm, acceleration_zout_norm
    return [acceleration_xout_norm, acceleration_yout_norm, acceleration_zout_norm]
    

bus = smbus.SMBus(1)
address = 0x68       # via i2cdetect
 
# Activate the smbus
bus.write_byte_data(address, power_mgmt_1, 0)

__update__();

if __name__ == "__main__":
    while(1):
        __update__();
        printvalues();
        time.sleep(1)
