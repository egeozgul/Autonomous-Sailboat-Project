from gyro import *
from time import sleep
import sys

calibration = "magnet_data.txt"
cal_str = "\r0.00% Calibrated"
time = 600;

if __name__ == "__main__":
    with open(calibration,'w') as f:
        f.write('');
    sys.stdout.write(cal_str)
    for i in range(time+1):
        sleep(.1);
        with open(calibration,"a") as f:
            x, y, z = AK8963_conv();
            f.write("{} {} {}\n".format(x,y,z));
        if (i%5 == 0):
            sys.stdout.flush();
            sys.stdout.write("\r{:.2f}% Calibrated".format(100*i/(time)));
    sys.stdout.write('\n')
