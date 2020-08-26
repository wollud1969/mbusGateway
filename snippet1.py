import wiringpi
from time import sleep
import serial

def init():
	wiringpi.wiringPiSetupGpio()
	wiringpi.pinMode(18, wiringpi.OUTPUT)
	wiringpi.pinMode(23, wiringpi.OUTPUT)
        wiringpi.pinMode(22, wiringpi.INPUT)
	wiringpi.pullUpDnControl(22, wiringpi.PUD_UP)
	wiringpi.pinMode(19, wiringpi.OUTPUT)
	wiringpi.pinMode(13, wiringpi.OUTPUT)
	wiringpi.pinMode(5, wiringpi.OUTPUT)
	wiringpi.pinMode(6, wiringpi.OUTPUT)
	loop(False)
	red(False)
	green(False)
	

def loop(v):
	if v:
		wiringpi.digitalWrite(18, 1)
		sleep(0.025)
		wiringpi.digitalWrite(18, 0)
	else:
		wiringpi.digitalWrite(23, 1)
		wiringpi.digitalWrite(23, 0)

def status():
	return wiringpi.digitalRead(22)

def green(v):
	if v:
		wiringpi.digitalWrite(6, 1)
	else:
		wiringpi.digitalWrite(6, 0)
		
def red(v):
	if v:
		wiringpi.digitalWrite(5, 1)
	else:
		wiringpi.digitalWrite(5, 0)
		
