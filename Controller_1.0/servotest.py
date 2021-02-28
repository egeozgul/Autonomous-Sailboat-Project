import RPi.GPIO as GPIO
import time
import sys;

PINA = 19 #GPIO 18 BOARD 12 #2 is 26
PINB = 18

GPIO.setmode(GPIO.BCM)

GPIO.setup(PINA, GPIO.OUT)
GPIO.setup(PINB, GPIO.OUT)

sail = GPIO.PWM(PINA, 50)
rudder = GPIO.PWM(PINB, 60)

sail.start(0)
rudder.start(0)

sailpos = 0;
rudderpos = 0;

def set_rudder(perc):
    global rudderpos;
    assert(perc >= 0 and perc <= 100);
    rudderpos = perc/100*7 + 7

def set_sail(perc):
    global sailpos;
    assert(perc >= 0 and perc <= 100);
    sailpos = perc;

def __update__():
  global sailpos, rudderpos;
  try:
    __update__();
  except KeyboardInterrupt:
    sail.stop()
    rudder.stop();
    GPIO.cleanup()

if __name__ == "__main__":
    try:
        targetpos = 0;
        if len(sys.argv) == 2:
            targetpos = float(sys.argv[1]);
        while(True):
            if (0):
                sailpos = targetpos
                sail.ChangeDutyCycle(sailpos);
            else:
                rudderpos = targetpos
                rudder.ChangeDutyCycle(rudderpos);
            break;
        sail.stop();
        rudder.stop();
        GPIO.cleanup();
    except KeyboardInterrupt:
        sail.stop()
        rudder.stop();
        GPIO.cleanup()
