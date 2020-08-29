import wiringpi
from time import sleep
import serial
from enum import Enum

LOOP_ENABLE = 18
LOOP_DISABLE = 23
LOOP_STATUS = 22

FRONTEND_RESET = 26
FRONTEND_SAMPLE_HOLD = 19
FRONTEND_RX_ENABLE = 13

LED_RED = 5
LED_GREEN = 6


def frontendReset():
  wiringpi.digitalWrite(FRONTEND_RESET, 0)
  sleep(0.025)
  wiringpi.digitalWrite(FRONTEND_RESET, 1)
  sleep(0.5)

def init():
  wiringpi.wiringPiSetupGpio()

  for p in (LOOP_ENABLE, LOOP_DISABLE, 
            FRONTEND_RESET, FRONTEND_RX_ENABLE, FRONTEND_SAMPLE_HOLD,
            LED_RED, LED_GREEN):
    wiringpi.pinMode(p, wiringpi.OUTPUT)
    wiringpi.digitalWrite(p, 0)

  wiringpi.pinMode(LOOP_STATUS, wiringpi.INPUT)
  wiringpi.pullUpDnControl(LOOP_STATUS, wiringpi.PUD_UP)

  frontendReset()

  loop(False)



def frontendSample():
  wiringpi.digitalWrite(FRONTEND_SAMPLE_HOLD, 0)


def frontendHold():
  wiringpi.digitalWrite(FRONTEND_SAMPLE_HOLD, 1)


def frontendRxEnable(v):
  if v:
    wiringpi.digitalWrite(FRONTEND_RX_ENABLE, 1)
  else:
    wiringpi.digitalWrite(FRONTEND_RX_ENABLE, 0)


def loop(v):
  if v:
    wiringpi.digitalWrite(LOOP_ENABLE, 1)
    sleep(0.025)
    wiringpi.digitalWrite(LOOP_ENABLE, 0)
  else:
    wiringpi.digitalWrite(LOOP_DISABLE, 1)
    wiringpi.digitalWrite(LOOP_DISABLE, 0)


def status():
  return wiringpi.digitalRead(LOOP_STATUS)


def green(v):
  if v:
    wiringpi.digitalWrite(LED_GREEN, 1)
  else:
    wiringpi.digitalWrite(LED_GREEN, 0)


def red(v):
  if v:
    wiringpi.digitalWrite(LED_RED, 1)
  else:
    wiringpi.digitalWrite(LED_RED, 0)
    

class MeterbusResponseStates(Enum):
  START1 = 1
  LENGTH1 = 2
  LENGTH2 = 3
  START2 = 4
  C_FIELD = 5
  A_FIELD = 6
  C_FIELD = 7
  USERDATA = 8
  CHKSUM = 9
  STOP = 10
  DONE = 99
  ERROR = 100

def a2h(a):
    return [ hex(x) for x in a ]

class MeterbusSerial(object):
  def __init__(self):
    self.port = serial.Serial('/dev/ttyAMA0', baudrate=2400, bytesize=8, parity='E', 
                              stopbits=1, timeout=None, xonxoff=0, rtscts=0)

  def shortFrameRequest(self, cmd, addr):
    chksum = (cmd + addr) & 0x00ff
    msg = bytearray([0x10, cmd, addr, chksum, 0x16])
    print(a2h(msg))

    frontendSample()
    frontendRxEnable(False)

    self.port.write(msg)

    # FIXME: Attention, the actual write time of the interface is not considered.
    #        This is a measured value.
    sleep(0.030)

    frontendHold()
    frontendRxEnable(True)

    frameData = []
    frame = {
      'l': 0,
      'c': 0,
      'a': 0,
      'ci': 0,
      'userdata': []
    }
    expectedUserDataOctets = 0
    state = MeterbusResponseStates.START1
    while (state not in [MeterbusResponseStates.DONE, MeterbusResponseStates.ERROR]):
      print("Waiting for input ... ")
      c = self.port.read()

      print("State {}, Octet {:02X}".format(state, c))



if __name__ == "__main__":
    init()
    loop(False)
    sleep(5.0)
    loop(True)
    sleep(5.0)
    m = MeterbusSerial()
    m.shortFrameRequest(0x5b, 84)


