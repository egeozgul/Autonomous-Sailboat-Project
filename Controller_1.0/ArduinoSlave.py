from smbus import SMBus
import time
from ast import literal_eval

global x, y, wind_angle, wind_speed, failed;
x = y = wind_angle = wind_speed = 0;
bus = SMBus(1)
address = 0x08
#time.sleep(3)

def get_values():
    global x, y, wind_speed, wind_speed;
    return [wind_angle, wind_speed, x, y];

def getx():
    global x;
    return x;

def gety():
    global y;
    return y;

def is_fail():
    global failed;
    return failed;

def __update__():
    global x,y, wind_angle, wind_speed, failed
    time.sleep(0.01)
    try:
        arr = bus.read_i2c_block_data(address, 0,32);
        if (arr[0] == arr[2] and arr[2] == arr[3]):
            return;
        wind_angle = arr[0];
        wind_speed = arr[1] / 5 * 2.4;
        """native is rev/.2s ; 1 rev/s = 2.4km/h"""
        x = arr[2]
        y = arr[3]
        failed = False;
    except:
        failed = True;
        pass;

def __sense__():
    while(True):
        __update__();

if __name__ == "__main__":
    while(True):
        __update__()
        [angle, speed, x, y] = get_values();
        print("Angle {}; Speed {}; X{}; Y{}".format(angle, speed, x, y))
