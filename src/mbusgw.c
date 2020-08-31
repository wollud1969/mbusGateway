#include <wiringPi.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>


#define LOOP_ENABLE 18
#define LOOP_DISABLE 23
#define LOOP_STATUS 22

#define FRONTEND_RESET 26
#define FRONTEND_SAMPLE_HOLD 19

#define LED_RED 5
#define LED_GREEN 6


typedef struct {
  uint8_t start1;
  uint8_t length1;
  uint8_t length2;
  uint8_t start2;
  uint8_t l;
  uint8_t c;
  uint8_t a;
  uint8_t ci;
  uint8_t *userdata;
  uint8_t chksum;
  uint8_t stop;
} t_longframe;

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

void msleep(uint32_t t) {
  usleep(t * 1000);
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


void init() {
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

  frontendReset();

  loopControl(false);

  // TODO: initialize serial interface
}


t_longframe *request(uint8_t cmd, uint8_t addr) {
  t_longframe *frame = (t_longframe*) malloc(sizeof(t_longframe));
  if (! frame) {
    fprintf(stderr, "Unable to allocate memory for frame\n");
    return NULL;
  }
  frame->userdata = NULL;

  uint8_t chksum = cmd + addr;

  frontendSample();

  // TODO: serial write 0x10 cmd addr chksum 0x16

  // TODO: wait for transmit finish
  /*
    buf = array.array('h', [0])
    while True:
      fcntl.ioctl(self.port.fileno(), termios.TIOCSERGETLSR, buf, 1)
      if buf[0] & termios.TIOCSER_TEMT:
        break
  */

  msleep(1);
  frontendHold();

  uint8_t userdataIdx = 0;
  uint8_t calculatedChksum = 0;
  t_state state = e_START1;

  while ((state != e_DONE) &&
         (state != e_ERROR) &&
         (state != e_TIMEOUT)) {
    fprintf(stderr, "Waiting for input ...\n");
    uint8_t c = 0; // TODO: read one octet from serial interface with timeout
    // TODO: Checkout whether a timeout occured, then set state to e_TIMEOUT
    // TODO: and issue 'continue' to go to the top of the loop

    fprintf(stderr, "State %d, Octet %02x\n", state, c);

    switch(state) {
    case e_START1:
      if (c == 0x68) {
        frame->start1 = c;
        state = e_LENGTH1;
      } else {
        fprintf(stderr, "Invalid start1 symbol %02x\n", c);
        state = e_ERROR;
      }
      break;
    case e_LENGTH1:
      if (c <= 3) {
        fprintf(stderr, "Length to small %02x\n", c);
        state = e_ERROR;
      } else {
        frame->length1 = c;
        frame->userdata = (uint8_t*) malloc(frame->length1 - 3);
        if (! frame->userdata) {
          fprintf(stderr, "Unable to allocate memory for userdata\n");
          state = e_ERROR;
        }
      }
      break;
    case e_LENGTH2:
      if (frame->length1 != c) {
        fprintf(stderr, "Invalid length2 %02x vs. %02x\n", frame->length1, c);
        state = e_ERROR;
      } else {
        frame->length2 = c;
      }
      break;
    case e_START2:
      if (c == 0x68) {
        frame->start2 = c;
        state = e_C_FIELD;
      } else {
        fprintf(stderr, "Invalid start2 symbol %02x\n", c);
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
        fprintf(stderr, "Invalid checksum %02x vs %02x\n", calculatedChksum, c);
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
        fprintf(stderr, "Invalid stop symbol %02x\n", c);
        state = e_ERROR;
      }
      break;
    default:
      fprintf(stderr, "Illegal state %d\n", state);
      state = e_ERROR;
      break;
    }
  }

  if ((state == e_ERROR) || (state == e_TIMEOUT)) {
    if (frame->userdata) {
      free(frame->userdata);
      frame->userdata = NULL;
    }
    free(frame);
    frame = NULL;
  }

  return frame;
}


    
    

      
      
      





int main(void) {
  init();


}


