import RPi.GPIO as GPIO
import time
import threading

interruptPin_A = 20;
interruptPin_B = 21;


GPIO.setmode(GPIO.BCM)
GPIO.setup(interruptPin_A, GPIO.IN)
GPIO.setup(interruptPin_B, GPIO.IN)

absoluteSteps = 0;
direction = 0;


global prev_A, prev_B, A, B
A = prev_A = 0;
B = prev_B = 0;

def interruptA(channel):
    global A, B, prev_A, prev_B, direction, absoluteSteps;

    prev_A = 0 + A

    if (channel != 0):
        A = 1;
    else:
        A = 0;

    if (A == prev_A):
        return;
    
    if (B == 1):
        if (A == 1):
            direction = 1;
        else:
            direction = -1;
    else:
        if (A == 1):
            direction = -1;
        else:
            direction = 1;
    
    absoluteSteps += direction;



def interruptB(channel):
    global A;
    global B;
    global absoluteSteps;
    global direction;

    prev_B = 0 + B;
    
    if (channel != 0):
        B = 1;
    else:
        B = 0;

    if (B == prev_B):
        return;

    if (A == 1):
        if (B == 1):
            direction = -1;
        else:
            direction = 1;
    else:
        if (B == 1):
            direction = 1;
        else:
            direction = -1;
    absoluteSteps += direction;

freq = 0
def opticalEncoderThread():
    global absoluteSteps;
    global delta;

    t = time.time()
    while (True):
        interruptB(GPIO.input(interruptPin_A))
        interruptA(GPIO.input(interruptPin_B))
        t2 = time.time()
        delta = (t2-t);
        t = t2;

def main():
    global absoluteSteps;
    # GPIO.add_event_detect(interruptPin_A, GPIO.BOTH, callback=interruptA)  # add rising edge detection on a channel
    #GPIO.add_event_detect(interruptPin_B, GPIO.BOTH, callback=interruptB)  # add rising edge detection on a channel

    t = threading.Thread(target=opticalEncoderThread)
    t.start()

    while (True):
        t = absoluteSteps/32*360
        print(absoluteSteps, delta*1000000)
        time.sleep(0.1)

main()
