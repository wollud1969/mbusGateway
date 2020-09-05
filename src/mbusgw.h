#ifndef _MBUSGW_H_
#define _MBUSGW_H_

#include <stdint.h>


#define LOOP_ENABLE 18
#define LOOP_DISABLE 23
#define LOOP_STATUS 22

#define FRONTEND_RESET 26
#define FRONTEND_SAMPLE_HOLD 19

#define LED_RED 5
#define LED_GREEN 6

#define DEFAULT_SERIAL_DEVICE "/dev/ttyAMA0"

#define BAUDRATE 2400

#define LINEMODE_CMD_PREFIX 0
#define LINEMODE_CMD_TERMINATE 0
#define LINEMODE_CMD_LOOP_SHUTDOWN 1

#define SUCCESS 0
#define ERROR_TIMEOUT 1
#define ERROR_LOOP_FAILURE 2
#define ERROR_TX_REG_UNACCESSIBLE 3
#define ERROR_OUT_OF_MEMORY__FRAME 4
#define ERROR_OUT_OF_MEMORY__USERDATA 5
#define ERROR_STATE_ENGINE__START1 10
#define ERROR_STATE_ENGINE__LENGTH1 11
#define ERROR_STATE_ENGINE__LENGTH2 12
#define ERROR_STATE_ENGINE__START2 13
#define ERROR_STATE_ENGINE__INVALID_CHKSUM 14
#define ERROR_STATE_ENGINE__STOP 15
#define ERROR_STATE_ENGINE__ILLEGAL_STATE 16
#define ERROR_STATE_ENGINE__UNKNOWN 17


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






#endif
