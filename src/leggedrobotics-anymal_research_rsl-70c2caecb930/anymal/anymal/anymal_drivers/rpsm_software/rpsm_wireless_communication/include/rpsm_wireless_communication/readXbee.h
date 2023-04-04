#ifndef READXBEE_H_   /* Include guard */
#define READXBEE_H_

 #ifdef __cplusplus
     extern "C" {
 #endif

#include <stdint.h>
#include <inttypes.h>

#define CRC16 0x8005

int xbee_init (char *port, struct termios *tty);

int xbee_read_byte(int fd, uint8_t *buffer);

int xbee_read (int fd, uint8_t *buffer, int length);

int xbee_write (int fd, uint8_t *buffer, int length);

uint16_t gen_crc16(uint8_t *data, int size);

int check_crc(uint16_t crc, uint8_t *data, int size);

uint16_t convertToUint16 (uint8_t higher, uint8_t lower);

float convertToFloat32 (uint8_t integer, uint8_t decimal);

void parse_button_state(uint8_t *buffer);

void xbee_close(int fd);

 #ifdef __cplusplus
     }
 #endif

#endif // READXBEE_H_