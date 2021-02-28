import RPi.GPIO as GPIO
import threading
import time

# GPIO Ports
LY = 23 
RX = 24

global x, y;

global prev_time1, active_time1, inactive_time1, min_time1, max_time1, duty_list1
global prev_time2, active_time2, inactive_time2, min_time2, max_time2, duty_list2

# initialize interrupt handlers
def init():

    global prev_time1, active_time1, inactive_time1, min_time1, max_time1, duty_list1
    global prev_time2, active_time2, inactive_time2, min_time2, max_time2, duty_list2

    duty_list1 = [0,0,0,0,0,0,0,0,0,0];
    duty_list2 = [0,0,0,0,0,0,0,0,0,0];

    prev_time1 = active_time1 = inactive_time1 = 0;
    prev_time2 = active_time2 = inactive_time2 = 0;
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)					# Use BCM mode
                                            # define the Encoder switch inputs
    GPIO.setup(LY, GPIO.IN) 				
    GPIO.setup(RX, GPIO.IN)
                                            # setup callback thread for the A and B encoder 
                                            # use interrupts for all inputs
    GPIO.add_event_detect(LY, GPIO.BOTH, callbacka)
    GPIO.add_event_detect(RX, GPIO.BOTH, callbackb)	
    return

def callbacka(pin):
    global LY
    if (pin == LY or True):
        if (GPIO.input(LY)):
            rising_interrupt1(pin)
        else:
            falling_interrupt1(pin)


def callbackb(pin):
    global RX;
    if (pin == RX or True):
        if (GPIO.input(RX)):
            rising_interrupt2(pin)
        else:
            falling_interrupt2(pin)
            

def rising_interrupt1(pin):
    global prev_time1, inactive_time1
    now = time.time()
    inactive_time1 = now - prev_time1
    prev_time1 = now
    return

def falling_interrupt1(pin):
    global prev_time1, active_time1, inactive_time1, duty_list1
    
    active_time1 = time.time() - prev_time1;

    duty_cycle = active_time1/(active_time1 + inactive_time1);

    duty_cycle *= 40
    duty_cycle -= 3.25

    duty_list1.pop()
    duty_list1.insert(0,duty_cycle);

    return

def rising_interrupt2(pin):
    global prev_time2, inactive_time2
    now = time.time()
    inactive_time2 = now - prev_time2
    prev_time2 = now
    return

def falling_interrupt2(pin):
    global prev_time2, active_time2, inactive_time2, duty_list2
    
    active_time2 = time.time() - prev_time2;

    duty_cycle = active_time2/(active_time2 + inactive_time2);

    duty_cycle *= 40
    duty_cycle -= 3.25

    duty_list2.pop()
    duty_list2.insert(0,duty_cycle);

    return

def get_val(duty_list):
    val = sum(duty_list)/len(duty_list)
    if (val > 0):
        val = min(val, 1);
    else:
        val = max(val, -1);
    return val;

def gety():
    global duty_list1
    return get_val(duty_list1)

def getx():
    global duty_list2
    return get_val(duty_list2)
                            # THAT'S IT

# Main loop. Demonstrate reading, direction and speed of turning left/rignt
def __sense__():						# for faster reading with locks
    global prev_time1, active_time1, inactive_time1, min_time1, max_time1, duty_list1
    global prev_time2, active_time2, inactive_time2, min_time2, max_time2, duty_list2

    duty_list1 = [0,0,0,0,0,0,0,0,0,0];
    duty_list2 = [0,0,0,0,0,0,0,0,0,0];

    prev_time1 = active_time1 = inactive_time1 = 0;
    prev_time2 = active_time2 = inactive_time2 = 0;
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)					# Use BCM mode
                                            # define the Encoder switch inputs
    GPIO.setup(LY, GPIO.IN) 				
    GPIO.setup(RX, GPIO.IN)
                                            # setup callback thread for the A and B encoder 
                                            # use interrupts for all inputs
    GPIO.add_event_detect(LY, GPIO.BOTH, callbacka)
    GPIO.add_event_detect(RX, GPIO.BOTH, callbackb)	

    while(1):
        time.sleep(0.5)
        print("x is {} and y is {}".format(getx(),gety()));