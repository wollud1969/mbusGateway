import wiringpi
from time import sleep
import serial
from enum import Enum
import array
import fcntl
import termios
import pprint



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
  CI_FIELD = 7
  USERDATA = 8
  CHKSUM = 9
  STOP = 10
  DONE = 99
  ERROR = 100
  TIMEOUT = 101

def a2h(a):
    return [ hex(x) for x in a ]

class MeterbusSerial(object):
  def __init__(self):
    self.port = serial.Serial('/dev/ttyAMA0', baudrate=2400, bytesize=8, parity='E', 
                              stopbits=1, timeout=1.0, xonxoff=0, rtscts=0)

  def open(self):
    loop(True)
    sleep(0.5)

  def close(self):
    loop(False)

  def shortFrameRequest(self, cmd, addr):
    chksum = (cmd + addr) & 0x00ff
    msg = bytearray([0x10, cmd, addr, chksum, 0x16])
    # print(a2h(msg))

    frontendSample()
    
    self.port.write(msg)

    buf = array.array('h', [0])
    while True:
      fcntl.ioctl(self.port.fileno(), termios.TIOCSERGETLSR, buf, 1)
      if buf[0] & termios.TIOCSER_TEMT:
        break

    sleep(0.001)

    frontendHold()
    
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
    while (state not in [MeterbusResponseStates.DONE, MeterbusResponseStates.ERROR, MeterbusResponseStates.TIMEOUT]):
      # print("Waiting for input ... ")
      c = self.port.read(1)
      if len(c) == 0:
        state = MeterbusResponseStates.TIMEOUT
        continue
      c = ord(c)

      # print("State {}, Octet 0x{:02X}".format(state, c))

      if state == MeterbusResponseStates.START1:
        if c == 0x68:
          frameData.append(c)
          state = MeterbusResponseStates.LENGTH1
        else:
          print('Invalid start1 symbol')
          state = MeterbusResponseStates.ERROR
      elif state == MeterbusResponseStates.LENGTH1:
        frameData.append(c)
        frame['l'] = c
        expectedUserDataOctets = frame['l'] - 3
        state = MeterbusResponseStates.LENGTH2
      elif state == MeterbusResponseStates.LENGTH2:
        frameData.append(c)
        if frame['l'] == c:
          state = MeterbusResponseStates.START2
        else:
          print('Invalid length 2 octet')
          state = MeterbusResponseStates.ERROR
      elif state == MeterbusResponseStates.START2:
        frameData.append(c)
        if c == 0x68:
          frameData.append(c)
          state = MeterbusResponseStates.C_FIELD
        else:
          print('Invalid start2 symbol')
          state = MeterbusResponseStates.ERROR
      elif state == MeterbusResponseStates.C_FIELD:
        frameData.append(c)
        frame['c'] = c
        state = MeterbusResponseStates.A_FIELD
      elif state == MeterbusResponseStates.A_FIELD:
        frameData.append(c)
        frame['a'] = c
        state = MeterbusResponseStates.CI_FIELD
      elif state == MeterbusResponseStates.CI_FIELD:
        frameData.append(c)
        frame['ci'] = c
        state = MeterbusResponseStates.USERDATA
      elif state == MeterbusResponseStates.USERDATA:
        frameData.append(c)
        frame['userdata'].append(c)
        expectedUserDataOctets -= 1
        if expectedUserDataOctets == 0:
          state = MeterbusResponseStates.CHKSUM
      elif state == MeterbusResponseStates.CHKSUM:
        frameData.append(c)
        responseChksum = c
        calculatedChksum = (frame['c'] + frame['a'] + frame['ci'] + sum(frame['userdata'])) & 0x00ff
        if responseChksum != calculatedChksum:
          print('Invalid checksum {} {}'.format(responseChksum, calculatedChksum))
          state = MeterbusResponseStates.ERROR
        else:
          state = MeterbusResponseStates.STOP
      elif state == MeterbusResponseStates.STOP:
        frameData.append(c)
        if c == 0x16:
          state = MeterbusResponseStates.DONE
        else:
          print('Invalid stop symbol')
          state = MeterbusResponseStates.ERROR
      else:
        print('Invalid state')
        state = MeterbusResponseStates.ERROR

    # print(a2h(frameData))

    res = {}
    if state == MeterbusResponseStates.DONE:
      res = {'status': 'OK', 'frame': frameData, 'c': frame['c'], 'a': frame['a'], 'ci': frame['ci'], 'userdata': frame['userdata']}
    else:
      res = {'status': 'ERROR', 'code': state }

    return res



if __name__ == "__main__":
    init()
    loop(False)
    sleep(2.0)

    m = MeterbusSerial()
    m.open()

    devices = [ 84, 87, 86, 85, 82, 81, 83, 80 ]
    stats = {
      'total': {
        'cycles': 0,
        'ok': 0,
        'error': 0
      },
      'devices': { x: { 'ok': 0, 'error': 0 } for x in devices }
    }
    


    pp = pprint.PrettyPrinter(indent=4)
    while True:
      for a in devices:
        r = m.shortFrameRequest(0x5b, a)
        stats['total']['cycles'] += 1
        if r['status'] == 'ERROR':
          stats['total']['error'] += 1
          stats['devices'][a]['error'] += 1
          print("Error for {}, last state was {}, restarting loop".format(a, r['code']))
          m.close()
          sleep(5)
          m.open()
        else:
          stats['total']['ok'] += 1
          stats['devices'][a]['ok'] += 1
        sleep(1)
      pp.pprint(stats)
      print("")
      sleep(15)



