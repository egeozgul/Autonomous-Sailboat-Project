import serial
import time
import string
import pynmea2

port="/dev/ttyAMA0"
ser=serial.Serial(port, baudrate=9600, timeout=1.5)
global newmsg, lat, lon;

def get_pos():
	global lat, lon;
	return [lat, lon];

def get_msg():
	global newmsg;
	return newmsg;

def print_pos():
	global lat, lon;
	gps = "Latitude=" + str(lat) + " and Longitude=" + str(lon)
	#print(gps)

def get_values():
	global lat, lon;
	line = "{},{}".format(str(lat),str(lon));
	return line;

def __update__():
	global newmsg, lat, lon;
	
	dataout = pynmea2.NMEAStreamReader()
	newdata=ser.readline().decode("ascii", errors='replace')
	#print(newdata);
	if newdata[0:6] == "$GPRMC":
		newmsg=pynmea2.parse(newdata)
		lat=newmsg.latitude
		lon=newmsg.longitude

def __sense__():
	while(1):
		__update__();

if __name__ == "__main__":
    __sense__();
