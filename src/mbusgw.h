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


#define SUCCESS 0
#define ERROR_TIMEOUT 1
#define ERROR_STATE_ENGINE 2
#define ERROR_LOOP_FAILURE 3
#define ERROR_TX_REG_UNACCESSIBLE 4
#define ERROR_OUT_OF_MEMORY 5


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