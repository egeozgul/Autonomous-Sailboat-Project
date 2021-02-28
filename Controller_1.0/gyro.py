import smbus
import math
import time
from math import atan, copysign, pi
from statistics import mean

# Register
power_mgmt_1 = 0x6b;
power_mgmt_2 = 0x6c;

global gyroscope_xout, gyroscope_yout, gyroscope_zout
global acceleration_xout, acceleration_yout, acceleration_zout
global acceleration_xout_norm, acceleration_yout_norm, acceleration_zout_norm
global magx,magy,magz, magdir;
global magxb, magyb, magzb;
global magxr, magyr, magzr;
#Magdir is horizontal plane theta in degrees, east is 0.


# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
#AK8963 registers
AK8963_ADDR   = 0x0C
AK8963_ST1    = 0x02
HXH          = 0x04
HYH          = 0x06
HZH          = 0x08
AK8963_ST2   = 0x09
AK8963_CNTL  = 0x0A
mag_sens = 4900.0 # magnetometer sensitivity: 4800 uT

# start I2C driver
bus = smbus.SMBus(1) # start comm with i2c bus
def AK8963_reader(register):
    # read magnetometer values
    low = bus.read_byte_data(AK8963_ADDR, register-1)
    high = bus.read_byte_data(AK8963_ADDR, register)
    # combine higha and low for unsigned bit value
    value = ((high << 8) | low)
    # convert to +- value
    if(value > 32768):
        value -= 65536
    return value

def AK8963_conv():
    # raw magnetometer bits

    loop_count = 0
    while 1:
        mag_x = AK8963_reader(HXH)
        mag_y = AK8963_reader(HYH)
        mag_z = AK8963_reader(HZH)

        # the next line is needed for AK8963
        if bin(bus.read_byte_data(AK8963_ADDR,AK8963_ST2))=='0b10000':
            break
        loop_count+=1
        
    #convert to acceleration in g and gyro dps
    m_x = (mag_x/(2.0**15.0))*mag_sens
    m_y = (mag_y/(2.0**15.0))*mag_sens
    m_z = (mag_z/(2.0**15.0))*mag_sens

    return m_x,m_y,m_z

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
    global magx,magy,magz, magdir;

    magx, magy, magz = AK8963_conv();

    magx -= magxb;
    magy -= magyb;
    magz -= magzb;

    magx /= magxr
    magy /= magyr
    magz /= magzr;

    if magx == 0:
        magdir = copysign(90,magy);
    else:
        magdir = atan(magy/magx) * 180 / (2 * pi);
        magdir *= 9
    if (magdir < 0):
        magdir += 360;

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

    line = "{:.5f} {:.5f} {:.5f} ".format(gyroscope_xout, 
                                    gyroscope_yout, gyroscope_zout);
    line += "{:.5f} {:.5f} {:.5f} ".format(acceleration_xout, 
                                    acceleration_yout, acceleration_zout); 
    line += "{:.5f} {:.5f} {:.5f} ".format(acceleration_xout_norm, 
                            acceleration_yout_norm, acceleration_zout_norm); 
    
    line += "{} {}".format(get_x_rotation(acceleration_xout_norm, 
                    acceleration_yout_norm, acceleration_zout_norm),
                    get_y_rotation(acceleration_xout_norm, 
                    acceleration_yout_norm, acceleration_zout_norm))

    line += " {:.5f}".format(get_temp());
    
    return line;

def printvalues():
    global gyroscope_xout, gyroscope_yout, gyroscope_zout
    global acceleration_xout, acceleration_yout, acceleration_zout
    global acceleration_xout_norm, acceleration_yout_norm, acceleration_zout_norm
    global magx, magy, magz, magdir;

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
    print('MAG: {} {} {}'.format(magx, magy, magz));
    print('MAG DIR: {}'.format(magdir));
    print('temp: {} *C'.format(get_temp()));


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

def MPU6050_start():
    # alter sample rate (stability)
    samp_rate_div = 0 # sample rate = 8 kHz/(1+samp_rate_div)
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, samp_rate_div)
    time.sleep(0.1)
    # reset all sensors
    bus.write_byte_data(MPU6050_ADDR,PWR_MGMT_1,0x00)
    time.sleep(0.1)
    # power management and crystal settings
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x01)
    time.sleep(0.1)
    #Write to Configuration register
    bus.write_byte_data(MPU6050_ADDR, CONFIG, 0)
    time.sleep(0.1)
    #Write to Gyro configuration register
    gyro_config_sel = [0b00000,0b010000,0b10000,0b11000] # byte registers
    gyro_config_vals = [250.0,500.0,1000.0,2000.0] # degrees/sec
    gyro_indx = 0
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, int(gyro_config_sel[gyro_indx]))
    time.sleep(0.1)
    #Write to Accel configuration register
    accel_config_sel = [0b00000,0b01000,0b10000,0b11000] # byte registers
    accel_config_vals = [2.0,4.0,8.0,16.0] # g (g = 9.81 m/s^2)
    accel_indx = 0                            
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, int(accel_config_sel[accel_indx]))
    time.sleep(0.1)
    # interrupt register (related to overflow of data [FIFO])
    bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)
    time.sleep(0.1)
    return gyro_config_vals[gyro_indx],accel_config_vals[accel_indx]

def read_raw_bits(register):
    # read accel and gyro values
    high = bus.read_byte_data(MPU6050_ADDR, register)
    low = bus.read_byte_data(MPU6050_ADDR, register+1)

    # combine higha and low for unsigned bit value
    value = ((high << 8) | low)
    
    # convert to +- value
    if(value > 32768):
        value -= 65536
    return value

def mpu6050_conv():
    # raw acceleration bits
    acc_x = read_raw_bits(ACCEL_XOUT_H)
    acc_y = read_raw_bits(ACCEL_YOUT_H)
    acc_z = read_raw_bits(ACCEL_ZOUT_H)
    
    # raw gyroscope bits
    gyro_x = read_raw_bits(GYRO_XOUT_H)
    gyro_y = read_raw_bits(GYRO_YOUT_H)
    gyro_z = read_raw_bits(GYRO_ZOUT_H)

    #convert to acceleration in g and gyro dps
    a_x = (acc_x/(2.0**15.0))*accel_sens
    a_y = (acc_y/(2.0**15.0))*accel_sens
    a_z = (acc_z/(2.0**15.0))*accel_sens

    w_x = (gyro_x/(2.0**15.0))*gyro_sens
    w_y = (gyro_y/(2.0**15.0))*gyro_sens
    w_z = (gyro_z/(2.0**15.0))*gyro_sens

    return a_x,a_y,a_z,w_x,w_y,w_z

def AK8963_start():
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x00)
    time.sleep(0.1)
    AK8963_bit_res = 0b0001 # 0b0001 = 16-bit
    AK8963_samp_rate = 0b0110 # 0b0010 = 8 Hz, 0b0110 = 100 Hz
    AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate # bit conversion
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,AK8963_mode)
    time.sleep(0.1)

def get_temp():
     t_val = read_raw_bits(TEMP_OUT_H) # uncomment to read temp
     temp = ((t_val)/333.87)+21.0 # uncomment and add below in return
     return temp;
    
def init_magb():
    global magxb, magyb, magzb;
    global magxr, magyr, magzr;
    with open("magnet_data.txt","r") as f:
        x = [];
        y = [];
        z = [];
        line = f.readline();
        while line != "":
            line = line.replace('\n','').split(' ');
            x.append(float(line[0]));
            y.append(float(line[1]));
            z.append(float(line[2]));
            line = f.readline();
        magxb = min(x);
        magyb = min(y);
        magzb = min(z);
        magxr = max(x)-min(x);
        magyr = max(y)-min(y);
        magzr = max(z)-min(z);

gyro_sens,accel_sens = MPU6050_start() # instantiate gyro/accel
bus.write_byte_data(0x68, 0x37, 0x22)
bus.write_byte_data(0x68, 0x38, 0x01)
AK8963_start() #
magxb = magyb = magzb = 0;
magxr = magyr = magzr = 0;
init_magb();
print("{}".format(bus.read_byte_data(0x68,0x37)))
__update__();

if __name__ == "__main__":
    while(1):
        print("magb {} {} {}".format(magxb, magyb, magzb))
        print("magr {} {} {}".format(magxr, magyr, magzr))
        __update__();
        printvalues();
        time.sleep(.3)
