import RPi.GPIO as GPIO
import threading
import time

# GPIO Ports
Y1 = 23  		#true		
X1 = 18  			    
Y2 = 22
X2 = 24


Rotary_counter = 0  			# Start counting from 0
Current_Y1 = 1					# Assume that rotary switch is not 
Current_X1 = 1					# moving while we init software

LockRotary = threading.Lock()		# create lock for rotary switch
	

# initialize interrupt handlers
def init():
	GPIO.setwarnings(True)
	GPIO.setmode(GPIO.BCM)					# Use BCM mode
											# define the Encoder switch inputs
	GPIO.setup(Y1, GPIO.IN) 				
	GPIO.setup(X1, GPIO.IN)
											# setup callback thread for the A and B encoder 
											# use interrupts for all inputs
	GPIO.add_event_detect(Y1, GPIO.BOTH, callback)							# NO bouncetime 
	return


def rising_interrupt(args):
	return

def falling_interrupt(args):
	return

# Main loop. Demonstrate reading, direction and speed of turning left/rignt
def main():						# for faster reading with locks

	init()										# Init interrupts, GPIO, ...
				
	while True :								# start test 
		time.sleep(0.1)
											


# start main demo function
main()

