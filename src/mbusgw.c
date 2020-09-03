#include <wiringPi.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdarg.h>

#include "mbusgw.h"


typedef enum {
  e_START1,
  e_LENGTH1,
  e_LENGTH2,
  e_START2,
  e_C_FIELD,
  e_A_FIELD,
  e_CI_FIELD,
  e_USERDATA,
  e_CHKSUM,
  e_STOP,
  e_DONE,
  e_ERROR,
  e_TIMEOUT
} t_state;


int serialFd;
bool verbose = false;
bool loopActiveFlag = false;

void msleep(uint32_t t) {
  usleep(t * 1000);
}


void log(const char *format, va_list ap) {
  va_start(ap, format);
  if (verbose) {
    vfprintf(stderr, format, ap);
  }
  va_end(ap);
}

void errlog(const char *format, va_list ap) {
  va_start(ap, format);
  vfprintf(stderr, format, ap);
  va_end(ap);
}


void ledRed(bool v) {
  if (v) {
    digitalWrite(LED_RED, HIGH);
  } else {
    digitalWrite(LED_RED, LOW);
  }
}

void ledGreen(bool v) {
  if (v) {
    digitalWrite(LED_GREEN, HIGH);
  } else {
    digitalWrite(LED_GREEN, LOW);
  }
}
void frontendReset() {
  digitalWrite(FRONTEND_RESET, LOW);
  msleep(25);
  digitalWrite(FRONTEND_RESET, HIGH);
  msleep(100);
}

void loopControl(bool v) {
  if (v) {
    digitalWrite(LOOP_ENABLE, HIGH);
    msleep(25);
    digitalWrite(LOOP_ENABLE, LOW);
  } else {
    digitalWrite(LOOP_DISABLE, HIGH);
    digitalWrite(LOOP_DISABLE, LOW);
  }
}

void frontendSample() {
  digitalWrite(FRONTEND_SAMPLE_HOLD, LOW);
}

void frontendHold() {
  digitalWrite(FRONTEND_SAMPLE_HOLD, HIGH);
}

void loopStatusISR() {
  loopActiveFlag = digitalRead(LOOP_STATUS) == LOW;
  if (! loopActiveFlag) {
    ledRed(true);
  }
}


void myExit(int e) {
  ledRed(false);
  ledGreen(false);
  loopControl(false);
  frontendSample();

  exit(e);
}


void init() {
  log("setting up gpios\n");

  wiringPiSetupGpio();

  pinMode(LOOP_ENABLE, OUTPUT);
  digitalWrite(LOOP_ENABLE, LOW);

  pinMode(LOOP_DISABLE, OUTPUT);
  digitalWrite(LOOP_DISABLE, LOW);

  pinMode(FRONTEND_RESET, OUTPUT);
  digitalWrite(FRONTEND_RESET, LOW);

  pinMode(FRONTEND_SAMPLE_HOLD, OUTPUT);
  digitalWrite(FRONTEND_SAMPLE_HOLD, LOW);  

  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);

  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, LOW);

  pinMode(LOOP_STATUS, INPUT);
  pullUpDnControl(LOOP_STATUS, PUD_UP);

  wiringPiISR(LOOP_STATUS, INT_EDGE_BOTH, loopStatusISR);

  frontendReset();

  loopControl(false);
}

int openSerial(char *serialDevice, uint32_t speedNum) {
  int fd = open(serialDevice, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    errlog("error %d opening serial device %s: %s\n",
           errno, serialDevice, strerror(errno));
    return -1;
  }

  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    errlog("error %d getting attributes for serial device %s: %s\n",
           errno, serialDevice, strerror(errno));
    return -1;
  }

  int speed;
  switch (speedNum) {
  case 1200:
    speed = B1200;
    break;
  case 2400:
    speed = B2400;
    break;
  default:
    errlog("speed %d not supported\n", speedNum);
    return -1;
  }
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(CRTSCTS | CSTOPB | PARODD);
  tty.c_cflag |= (CLOCAL | CREAD | PARENB);

  tty.c_lflag = 0;

  tty.c_oflag = 0;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 50;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag |= IGNBRK;
  
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    errlog("error %d setting attributes for serial device %s: %s\n",
           errno, serialDevice, strerror(errno));
    return -1;
  }

  loopControl(true);
  msleep(2000);

  return fd;
}

void closeSerial(int fd) {
  loopControl(false);
  close(fd);
}

t_longframe *request(int fd, uint8_t cmd, uint8_t addr) {
  t_longframe *frame = (t_longframe*) malloc(sizeof(t_longframe));
  if (! frame) {
    errlog("unable to allocate memory for frame\n");
    return NULL;
  }
  frame->userdata = NULL;

  uint8_t chksum = cmd + addr;

  frontendSample();

  uint8_t sendBuf[5];
  sendBuf[0] = 0x10;
  sendBuf[1] = cmd;
  sendBuf[2] = addr;
  sendBuf[3] = chksum;
  sendBuf[4] = 0x16;
  write(fd, sendBuf, 5);

  while (1) {
    int r;
    if (ioctl(fd, TIOCSERGETLSR, &r) == -1) {
      errlog("error %d getting TIOCSERGETLSR for fd %d: %s\n",
             errno, fd, strerror(errno));
      errno = ERROR_APP_SPECIFIC_ERROR_FLAG | ERROR_TX_REG_UNACCESSIBLE;
      return NULL;
    }
    if (r & TIOCSER_TEMT) {
      break;
    }
  }

  msleep(1);
  frontendHold();

  uint8_t userdataIdx = 0;
  uint8_t calculatedChksum = 0;
  t_state state = e_START1;

  while ((state != e_DONE) &&
         (state != e_ERROR) &&
         (state != e_TIMEOUT)) {
     
    log("waiting for input ...\n");
    uint8_t c;
    ssize_t s = read(fd, &c, 1);
    if (s == 0) {
      errlog("timeout waiting for input\n");
      state = e_TIMEOUT;
      continue;
    }

    log("state %d, Octet %02x\n", state, c);

    switch(state) {
    case e_START1:
      if (c == 0x68) {
        frame->start1 = c;
        state = e_LENGTH1;
      } else {
        errlog("invalid start1 symbol %02x\n", c);
        state = e_ERROR;
      }
      break;
    case e_LENGTH1:
      if (c <= 3) {
        errlog("length to small %02x\n", c);
        state = e_ERROR;
      } else {
        frame->length1 = c;
        frame->userdata = (uint8_t*) malloc(frame->length1 - 3);
        if (! frame->userdata) {
          errlog("unable to allocate memory for userdata\n");
          state = e_ERROR;
        }
        state = e_LENGTH2;
      }
      break;
    case e_LENGTH2:
      if (frame->length1 != c) {
        errlog("invalid length2 %02x vs. %02x\n", frame->length1, c);
        state = e_ERROR;
      } else {
        frame->length2 = c;
        state = e_START2;
      }
      break;
    case e_START2:
      if (c == 0x68) {
        frame->start2 = c;
        state = e_C_FIELD;
      } else {
        errlog("invalid start2 symbol %02x\n", c);
        state = e_ERROR;
      }
      break;
    case e_C_FIELD:
      frame->c = c;
      calculatedChksum += c;
      state = e_A_FIELD;
      break;
    case e_A_FIELD:
      frame->a = c;
      calculatedChksum += c;
      state = e_CI_FIELD;
      break;
    case e_CI_FIELD:
      frame->ci = c;
      calculatedChksum += c;
      state = e_USERDATA;
      break;
    case e_USERDATA:
      frame->userdata[userdataIdx] = c;
      calculatedChksum += c;
      userdataIdx++;
      if (userdataIdx == (frame->length1 - 3)) {
        state = e_CHKSUM;
      }
      break;
    case e_CHKSUM:
      if (c != calculatedChksum) {
        errlog("invalid checksum %02x vs %02x\n", calculatedChksum, c);
        state = e_ERROR;
      } else {
        frame->chksum = c;
        state = e_STOP;
      }
      break;
    case e_STOP:
      if (c == 0x16) {
        frame->stop = c;
        state = e_DONE;
      } else {
        errlog("invalid stop symbol %02x\n", c);
        state = e_ERROR;
      }
      break;
    default:
      errlog("illegal state %d\n", state);
      state = e_ERROR;
      break;
    }
  }

  if ((state == e_ERROR) || (state == e_TIMEOUT)) {
    if (state == e_ERROR) {
      errno = ERROR_TX_REG_UNACCESSIBLE | ERROR_STATE_ENGINE;
    } else if (state == e_TIMEOUT) {
      errno = ERROR_TX_REG_UNACCESSIBLE | ERROR_TIMEOUT;
    }
    if (frame->userdata) {
      free(frame->userdata);
      frame->userdata = NULL;
    }
    free(frame);
    frame = NULL;
  }

  return frame;
}

void printFrame(bool hexOut, t_longframe *frame) {
  if (hexOut) {
    fprintf(stderr, "%02x %02x %02x %02x %02x %02x %02x\n",
            frame->start1, frame->length1, frame->length2, frame->start2,
            frame->c, frame->a, frame->ci);
    for (uint8_t i = 0; i < (frame->length1 - 3); i++) {
      if (i && !(i % 16)) {
        fprintf(stderr, "\n");
      }
      fprintf(stderr, "%02x ", frame->userdata[i]);
    }
    fprintf(stderr, "\n");
    fprintf(stderr, "%02x %02x\n", frame->chksum, frame->stop);
  } else {
    fprintf(stdout, "%c%c", 0, frame->length1 + 6);
    fprintf(stdout, "%c%c%c%c%c%c%c",
            frame->start1, frame->length1, frame->length2, frame->start2,
            frame->c, frame->a, frame->ci);
    for (uint8_t i = 0; i < (frame->length1 - 3); i++) {
      fprintf(stdout, "%c", frame->userdata[i]);
    }
    fprintf(stdout, "%c%c", frame->chksum, frame->stop);
    fflush(stdout);
  }
}

int main(int argc, char *argv[]) {
  init();

  ledGreen(true);

  int exitCode = 0;
  bool hexOut = false;
  bool lineMode = false;
  uint8_t addr = 0;
  uint8_t cmd = 0x5b;

  int opt;
  while ((opt = getopt(argc, argv, "lhvxc:a:")) != -1) {
    switch(opt) {
    case 'h':
      errlog("mbusgw - interface access tool for meterbus gateway\n");
      errlog("-h      ... Show this help page\n");
      errlog("-v      ... Verbose output\n");
      errlog("-x      ... Output as hex string in 'human-readable' form\n");
      errlog("-a addr ... Address of device to be queried\n");
      errlog("-c cmd  ... Command to be sent, default is 0x5b\n");
      errlog("-l      ... Read cmd and addr from stdin unless\n");
      errlog("            0x00 0x00 is provided and return the\n");
      errlog("            result on stdout.\n");
      errlog("            It is preceeds by 0x00 for okay and the\n");
      errlog("            length.\n");
      errlog("            In case of an error it will by 0xff followed\n");
      errlog("            by one octet with an error code\n");
      errlog("\n");
      break;
    case 'v':
      verbose = true;
      break;
    case 'l':
      lineMode = true;
      break;
    case 'x':
      hexOut = true;
      break;
    case 'a':
      addr = (uint8_t) strtol(optarg, NULL, 0);
      break;
    case 'c':
      cmd = (uint8_t) strtol(optarg, NULL, 0);
      break;
    }
  }


  log"opening device\n");
  int fd = openSerial(DEFAULT_SERIAL_DEVICE, 2400);
  if (fd == -1) {
    errlog("unable to open device, fatal error\n");
    myExit(-1);
  }


  while (1) {
    if (! loopActiveFlag) {
      errlog("loop is not active, enable it and delay\n");
      loopControl(true);
      msleep(2000);
    }

    if (lineMode) {
      log("lineMode, waiting for input\n");
      fread(&cmd, 1, 1, stdin);
      fread(&addr, 1, 1, stdin);
    }
    if ((cmd == 0) && (addr == 0)) {
      errlog(stderr, "termination requested\n");
      break;
    }

    log("sending request %02x %02x\n", cmd, addr);
    t_longframe *frame = NULL;
    if (loopActiveFlag) {
      ledRed(false);
      frame = request(fd, cmd, addr);
    } else {
      errlog("loop is currently inactive, no need to try\n");
    }

    if (frame) {
      log("received a valid frame\n");
      printFrame(hexOut, frame);
      free(frame->userdata);
      frame->userdata = NULL;
      free(frame);
      frame = NULL;
    } else {
      ledRed(true);
      if (! loopActiveFlag) {
        errno = ERROR_APP_SPECIFIC_ERROR_FLAG | ERROR_LOOP_FAILURE;
      }
      errlog("error %04x occured\n", errno);
      if (! hexOut) {
        uint8_t maskedError = (uint8_t)(errno & ~ERROR_APP_SPECIFIC_ERROR_FLAG);
        fprintf(stdout, "%c%c", maskedError, 0);
        fflush(stdout);
      }
      if (! lineMode) {
        exitCode = -2;
      }
    }

    if (! lineMode) {
      break;
    }
  }

  log("closing device\n");
  closeSerial(fd);

  myExit(exitCode);
}





