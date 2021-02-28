from smbus import SMBus
import time
bus = SMBus(1)
address = 0x08
time.sleep(3)

data = [0xA5,0x5A]
wind_angle = 0;

def update_angle():
	global wind_angle;
	try:
		wind_angle = bus.read_byte_data(address, 0)
	except:
		time.sleep(0.01);
		return get_data();
	return wind_angle;

def sense():
	while(True):
		update_angle;