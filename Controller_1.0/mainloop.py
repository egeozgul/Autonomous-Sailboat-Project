import gyro
import gpstest as gps
import ArduinoSlave as mc;
import threading
import time
import servotest as servo;
from model_static import sail
from os import path

def log_sensor_data():
    prev = time.time();
    output = "sensorlogs/"
    #Round down to previous 5 minutes.
    prev = prev - prev%300;
    if (not path.exists(output + "{:.0f}.txt".format(prev))):
        with open(output + "{:.0f}.txt".format(prev),'w') as f:
            f.write("windangle windspeed sailpos rudderpos\tgyrox gyroy gyroz accelx accely accelz accelxnorm accelynorm accelznorm xrot yrot temperature\tlat lon\n")
    with open(output + "{:.0f}.txt".format(prev),'a') as f:
        line = ""
        for i in mc.get_values():
            line += str(i) + " "
        line += "\t"
        line += gyro.get_values() + "\t"
        line += gps.get_values();
        f.write(line + '\n')

gyro_t = threading.Thread(target=gyro.__sense__, daemon=True);
gps_t = threading.Thread(target=gps.__sense__, daemon=True);
mc_t = threading.Thread(target=mc.__sense__, daemon=True);

gyro_t.start();
gps_t.start();
mc_t.start();

manual = True;

while(1):
    time.sleep(5)
    #gyro.printvalues();
    #gpstest.print_pos();
    if (manual):
        x = mc.getx()
        y = mc.gety()
        servo.set_rudder(y/255*100);
        #servo.set_sail(x/255*100);
        log_sensor_data();
    else:
        wind_dir = mc.get_values()[0];
        boat_angle = gyro.get_angle();
        target_lat = -75.477791
        target_lon = 40.613953
        [r, s] = sail(gps.lat, gps.lon, target_lat, target_lon, wind_dir, boat_angle);
        servo.set_rudder(r/18*10)
        #servo.set_sail(s);
